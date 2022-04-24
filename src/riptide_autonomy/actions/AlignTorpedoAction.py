#! /usr/bin/env python3

from ast import AsyncFunctionDef
from math import cos, pi, sin, sqrt, tan
from queue import Queue

import cv2
import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_system_default
from cv_bridge import CvBridge
from rclpy.node import Node
from riptide_msgs2.action import AlignTorpedos
from sensor_msgs.msg import CompressedImage
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf_transformations import euler_from_quaternion

import utils.TorpedoVision as TorpedoVision

IMAGE_TOPIC = "stereo/left/image_color/compressed"

# #TODO: figure out to delete because we cannot reliably see the prop. if needed, figure out these values
# PROP_SIZE_x = 10 #meters
# PROP_SIZE_y = 10 #also meters

ROBOT_NAME = "tempest"
CAMERA_FOV_X = 90 #degrees
CAMERA_FOV_Y = 45 #degrees

class AlignTorpedosService(Node):
    
    def __init__(self):
        super().__init__('AlignTorpedos')
        self.actionServer = ActionServer(self, AlignTorpedos, 'AlignTorpedos', self.alignTorpedosCallback)
        self.cvBridge = CvBridge()
        
        #tf stuff
        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer, self)
        
        #subscribers
        self.imageSub = self.create_subscription(CompressedImage, IMAGE_TOPIC, self.imageCallback, qos_profile_system_default)
        self.imgQueue = Queue(1)
        
    #
    # Utils
    #
    
    #TODO: make a util file to store all of these util methods. when you make one, change away from using TorpedoVision.dist which can only do 2d points
    def dist(self, pt1, pt2):
        if not (len(pt1) == len(pt2)):
            self.get_logger().error("Cannot calculate distance between points of differing dimensions!")
            return -1

        sum = 0
        for i in range(0, len(pt1)):
            diff = pt2[i] - pt1[i]
            sum += diff**2 #diff squared
        
        return sqrt(sum)
    
    
    def closestToPoint(self, rects, target):
        dist = 99999999999
        closest = None
        for (x, y, w, h) in rects:
            d = TorpedoVision.dist((x, y), target)
            if d < dist:
                dist = d
                closest = (x, y, w, h)
        
        return closest
    
    
    def averageOfTuples(self, points):
        res = [0] * len(points[0]) #array with same size as points
        for point in points:
            for i in range(0, len(point)):
                res[i] += point[i]
                
        for comp in res:
            comp /= len(points)
            
        return res
    
    
    def centerOfRect(self, rect):
        x, y, w, h = rect
        cX = x + (w/2)
        cY = y + (h/2)
        
        return (cX, cY)
            
    
    #TODO: maybe put a link to the picture of the math I did to get the formula for this
    def pixelsToMeters(self, px, imgLen, fov):
        """Converts a measure of pixels in the direction of an axis (x, y) to meters.

        Args:
            px (int): Pixels to be converted to meters
            imgLen (int): if px is in the x-direction, use the image width. if px is y direction, use image height.
            fov (int): in degrees. if px is in the x-direction, use the horizontal fov. if y, use vertical.

        Returns:
            double: px converted to meters
        """
        
        fovDeg = fov * (pi / 180.0)
        return px / tan((px * imgLen) / fovDeg)
    
    #basically this method should be the exact python version of doTransform() in util.cpp but without transforming the quaternion and returning a pose
    def transformPosition(self, coords, transform: TransformStamped) -> Vector3:
        #rotate point with transform orientation
        rpy = euler_from_quaternion(transform.transform.rotation)
        yaw = rpy[2]
        
        x = coords[0] * cos(yaw) - coords[1] * sin(yaw)
        y = coords[0] * sin(yaw) + coords[1] * cos(yaw)
        
        position = Vector3()
        position.x = x + transform.transform.translation.x
        position.y = y + transform.transform.translation.y
        position.z = coords[2] + transform.transform.translation.z
        
        return position
    
    
    #transforms image pixel coordinates to global coordinates in world frame. returns (-1000, -1000, -1000) if an error happens.
    def imageToGlobalCoordinates(self, center, distance, imgShape):
        cX, cY = center
        imgCenX = imgShape[0] / 2
        imgCenY = imgShape[1] / 2
        
        #move image coordinate frame so that (0, 0) is the center of the image
        cX -= imgCenX
        cY -= imgCenY
        
        #convert x and y to meters. This (with the distance) will be the location of the object in camera frame.
        xMeters = self.pixelsToMeters(cX, imgShape[0], CAMERA_FOV_X)
        yMeters = self.pixelsToMeters(cY, imgShape[1], CAMERA_FOV_Y)
        
        #now all we need to do is convert that to world frame
        try:
            toFrame = "world"
            fromFrame = ROBOT_NAME + "/stereo/left_link"
            now = self.get_clock().now()
            
            cameraToWorld = self.tfBuffer.lookup_transform(
                toFrame,
                fromFrame,
                now
            )
            
            #transform point (distance, xMeters, yMeters). distance is first because it is forwards/backwards, x is second because its left/right, y third because up/down
            globalCoords = self.transformPosition((distance, xMeters, yMeters), cameraToWorld)
            return globalCoords
        except TransformException as ex:
            self.get_logger().warn("Could not transform {} to {}! {}".format(toFrame, fromFrame, ex))
        
        return (-1000, -1000, -1000) # error coords error coords
        
        
    #
    # Callbacks and ROS stuff
    #
        
    
    def imageCallback(self, msg):
        img = self.cvBridge.compressed_imgmsg_to_cv2(msg)
        if self.imgQueue.full():
            self.imgQueue.get_nowait()
        
        self.imgQueue.put_nowait(img)
        
    
    def waitForImage(self, timeout=3.0) -> np.ndarray:
        return self.imgQueue.get(True, timeout)
        
        
    def alignTorpedosCallback(self, goalHandle):
        self.get_logger().info("Starting align torpedos action.")
        distance = goalHandle.request.distance
        torpedo = goalHandle.request.torpedo
        timeout = goalHandle.request.timeoutms
        
        startTime = self.get_clock().now()
        
        #list that stores lists of target coordinates. (2d list! wow!)
        upperCoords = [] #location of circles in world frame
        lowerCoords = [] #location of special shapes in world frame
        
        # collect 25 good detections or time out after the specified timeout time
        while (len(upperCoords) < 25 or len(lowerCoords) < 25) and (self.get_clock().now() - startTime).to_msg().sec * 1000.0 < timeout:
            img = self.waitForImage()
                        
            if img is not None:                
                holes = TorpedoVision.processImage(img)
                
                #find top leftmost and bottom rightmost point
                topLeftmost = self.closestToPoint(holes, (0, 0))
                bottomRightmost = self.closestToPoint(holes, (img.shape[0], img.shape[1]))
                
                topLeftmostWorld = self.imageToGlobalCoordinates(self.centerOfRect(topLeftmost), distance, img.shape)
                bottomRightmostWorld = self.imageToGlobalCoordinates(self.centerOfRect(bottomRightmost), distance, img.shape)
                
                #put each coordinate in its respective list
                if topLeftmost != bottomRightmost:
                    upperCoords.append(topLeftmostWorld)
                    lowerCoords.append(bottomRightmostWorld)
                else: #since they are equal, put it in the list that it is closest to
                    upperAvg = self.averageOfTuples(upperCoords)
                    lowerAvg = self.averageOfTuples(lowerCoords)
                    
                    if self.dist(upperAvg, topLeftmostWorld) < self.dist(lowerAvg, topLeftmostWorld):
                        upperCoords.append(topLeftmostWorld)
                    else:
                        lowerCoords.append(topLeftmostWorld)
                                
                for [x, y, w, h] in holes:
                    cv2.circle(img, (int(x + (w/2)), int(y + (h/2))), 2, (0, 255, 0), 2)
                    
                cv2.imshow("result", img)
                cv2.waitKey(1)
        
        #analyze the target coordinates to figure out where the targets are in global space and which one to go to
        upperPoint = self.averageOfTuples(upperCoords)
        lowerPoint = self.averageOfTuples(lowerCoords)
                
        goalHandle.succeed()
        self.get_logger().info("Torpedo alignment complete.")
        return AlignTorpedos.Result()
    
    
def main(args=None):
    rclpy.init(args=args)
    
    service = AlignTorpedosService()
    
    service.get_logger().info("Starting Align Torpedo service client.")
    rclpy.spin(service, executor=MultiThreadedExecutor())
    rclpy.shutdown()
    
    
#program starts here
if __name__ == '__main__':
    main()
    