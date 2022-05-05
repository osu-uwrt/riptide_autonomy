#! /usr/bin/env python3

from math import cos, pi, sin, sqrt, tan
from queue import Queue

import cv2
import numpy as np
import rclpy
import rclpy.time
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_system_default
from cv_bridge import CvBridge
from rclpy.node import Node
from riptide_msgs2.action import AlignTorpedos
from sensor_msgs.msg import Image
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf_transformations import euler_from_quaternion
from statistics import pstdev

import utils.TorpedoVision as TorpedoVision

IMAGE_TOPIC = "/tempest/stereo/left/image_rect_color" #TODO: make this not an absolute namespace

# #TODO: figure out to delete because we cannot reliably see the borders of the prop. if needed, figure out these values
# PROP_SIZE_x = 10 #meters
# PROP_SIZE_y = 10 #also meters

ROBOT_NAME = "tempest"
CAMERA_FOV_X = 81.25 #degrees
CAMERA_FOV_Y = 45.7 #degrees

GROUP_SIZE = 0.0625
NON_CIRCLE_PRIORITY = 1.3 #measure of how much more "important" non-circle shapes are. this will affect how the alignment prioritizes the special shapes for more points

class AlignTorpedosService(Node):
    
    def __init__(self):
        super().__init__('AlignTorpedos')
        self.actionServer = ActionServer(self, AlignTorpedos, 'AlignTorpedos', self.alignTorpedosCallback)
        self.cvBridge = CvBridge()
        
        #tf stuff
        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer, self)
        
        #subscribers
        self.imageSub = self.create_subscription(Image, IMAGE_TOPIC, self.imageCallback, qos_profile_system_default)
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
            d = self.dist((x, y), target)
            if d < dist:
                dist = d
                closest = (x, y, w, h)
        
        return closest
    
    
    def averageOfTuples(self, points, tupleSize):
        if len(points) == 0:
            return [0.0] * tupleSize
               
        res = [0.0] * tupleSize
        for point in points:
            for i in range(0, len(point)):
                res[i] += point[i]
                
        for i in range(0, len(res)):
            res[i] /= len(points)
        
        # for comp in res:
        #     comp /= len(res)
            
        return res
    
    
    def centerOfRect(self, rect):
        x, y, w, h = rect
        cX = x + (w/2)
        cY = y + (h/2)
        
        return (cX, cY)
    
    
    def filterPoints(self, points, tupleSize):
        if len(points) == 0:
            return points
        
        avg = self.averageOfTuples(points, tupleSize)
        
        #get distance of each point from average point
        dists = [self.dist(avg, pt) for pt in points]
        avgDist = sum(dists) / len(dists)
        stdev = pstdev(dists) #standard deviation
        
        # filter out all points that are more than a certain number of stdevs away from avg
        goodPts = []
        for i in range(0, len(dists)):
            if abs(dists[i] - avgDist) < (stdev * 1.3):
                goodPts.append(points[i])
        
        return goodPts
        
            
    
    #TODO: maybe put a link to the picture of the math I did to get the formula for this
    def pixelsToMeters(self, px, imgLen, fov, dist):
        """Converts a measure of pixels in the direction of an axis (x, y) to meters.

        Args:
            px (int): Pixels to be converted to meters
            imgLen (int): if px is in the x-direction, use the image width. if px is y direction, use image height.
            fov (int): in degrees. if px is in the x-direction, use the horizontal fov. if y, use vertical.

        Returns:
            double: px converted to meters
        """
        if px == 0:
            return dist * sin(fov / 2)
        
        fovDeg = fov * (pi / 180.0)
        distPx = px / tan((px / imgLen) / fovDeg)
        pixelsPerMeter = distPx / dist
        return px / pixelsPerMeter
    
    #basically this method should be the exact python version of doTransform() in util.cpp but without transforming the quaternion and returning a pose
    def transformPosition(self, coords, transform: TransformStamped) -> Vector3:
        #rotate point with transform orientation
        
        rpy = euler_from_quaternion([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ])
        
        yaw = rpy[2]
        
        x = coords[0] * cos(yaw) - coords[1] * sin(yaw)
        y = coords[0] * sin(yaw) + coords[1] * cos(yaw)
        
        position = Vector3()
        position.x = x + transform.transform.translation.x
        position.y = y + transform.transform.translation.y
        position.z = coords[2] + transform.transform.translation.z
        
        return position
    
    
    #transforms image pixel coordinates to global coordinates in world frame. returns (-1000, -1000, -1000) if an error happens.
    def imageToWorldCoordinates(self, center, distance, imgShape):
        cX, cY = center
        imgCenX = imgShape[0] / 2
        imgCenY = imgShape[1] / 2
        
        #move image coordinate frame so that (0, 0) is the center of the image
        cX -= imgCenX
        cY -= imgCenY
        
        #convert x and y to meters. This (with the distance) will be the location of the object in camera frame.
        xMeters = self.pixelsToMeters(cX, imgShape[0], CAMERA_FOV_X, distance)
        yMeters = self.pixelsToMeters(cY, imgShape[1], CAMERA_FOV_Y, distance)
        
        #now all we need to do is convert that to world frame
        try:
            toFrame = "world"
            # fromFrame = "tempest_left_camera_frame"
            fromFrame = ROBOT_NAME + "/stereo/left_link"
            
            cameraToWorld = self.tfBuffer.lookup_transform(
                toFrame,
                fromFrame,
                rclpy.time.Time()
            )
            
            #transform point (distance, xMeters, yMeters). distance is first because it is forwards/backwards, x is second because its left/right, y third because up/down
            globalCoords = self.transformPosition((distance, xMeters, yMeters), cameraToWorld)
            return [globalCoords.x, globalCoords.y, globalCoords.z]
        except TransformException as ex:
            self.get_logger().warn("Could not transform {} to {}! {}".format(toFrame, fromFrame, ex))
        
        return (-1000.0, -1000.0, -1000.0) # error coords error coords
        
        
    #
    # Callbacks and ROS stuff
    #
        
    
    def imageCallback(self, msg):
        img = self.cvBridge.imgmsg_to_cv2(msg)
        if self.imgQueue.full():
            self.imgQueue.get_nowait()
        
        self.imgQueue.put_nowait(img)
        
    
    def waitForImage(self, timeout=3.0) -> np.ndarray:
        return self.imgQueue.get(True, timeout)
        
        
    def alignTorpedosCallback(self, goalHandle):
        self.get_logger().info("Starting align torpedos action.")
        distance = goalHandle.request.distance
        timeout = goalHandle.request.timeoutms
        
        startTime = self.get_clock().now()
        
        #list that stores lists of target coordinates. (2d list! wow!)
        coords = [] #location of circles in world frame, paired with number of segments of that shape
        
        # collect 25 good detections or time out after the specified timeout time
        while (len(coords) < 50) and (self.get_clock().now() - startTime).to_msg().sec * 1000.0 < timeout:
            img = self.waitForImage()
                        
            if img is not None:
                #TODO: maybe instead of looking at corners, look at circularity instead
                #peep this link: https://riptutorial.com/opencv/example/22518/circular-blob-detection
                
                holes = TorpedoVision.processImage(img) #returns a list of (contour, numCorners)
                rects = [(cv2.boundingRect(hole), corners) for hole, corners in holes]
                print(rects)
                
                for ((x, y, w, h), segments) in rects:
                    globalCoords = self.imageToWorldCoordinates((x, y), distance, img.shape)
                    coords.append((globalCoords, segments))
                        
                for ((x, y, w, h), segments) in rects:
                    cv2.circle(img, (int(x + (w/2)), int(y + (h/2))), 2, (0, 255, 0), 2)
                    
                cv2.imshow("result", img)
                cv2.waitKey(1)
        
        print()
        print()
        
        # group all detections into groups of detections that are close to each other. The radius of each group is determined by GROUP_SIZE
        groups = [] #will be a list of (groups of coordinates that are close together, numSegments)
        for (coord, segments) in coords:
            #find all groups that the coordinate belongs to
            foundGroup = False
            for i in range(0, len(groups)):
                (group, groupSegments) = groups[i]
                if self.dist(coord, self.averageOfTuples(group, 3)) < GROUP_SIZE and segments == groupSegments:
                    groups[i][0].append(coord)
                    foundGroup = True

            if not foundGroup:
                groups.append(([coord], segments))
        
        #find the average coordinate of each group
        finalCoords = [] #will be list of (coord, numDetections, numSegments)
        for (coords, segments) in groups:
            finalCoords.append((self.averageOfTuples(coords, 3), len(coords), segments))
            
        print()
        print()
        print("RESULTS:")
        for coord in finalCoords:
            print(coord)
            
        #now score each group based on the shape and number of detections (circle is lower score and more detections is better)
        scores = []
        for (coord, numDetections, numSegments) in finalCoords:
            multiplier = NON_CIRCLE_PRIORITY if numSegments != 8 else 1
            scores.append(numDetections * multiplier)
            
        #find coordinate to align to by using the one with the highest score
        index = scores.index(max(scores)) #index of max of scores
        targetCoord = finalCoords[index][0]
            
        actionResult = AlignTorpedos.Result()
        actionResult.coords.x = targetCoord[0]
        actionResult.coords.y = targetCoord[1]
        actionResult.coords.z = targetCoord[2]
        
        goalHandle.succeed()
        self.get_logger().info("Torpedo alignment complete.")
        return actionResult
    
    
def main(args=None):
    rclpy.init(args=args)
    
    service = AlignTorpedosService()
    
    service.get_logger().info("Starting Align Torpedo service client.")
    rclpy.spin(service, executor=MultiThreadedExecutor())
    rclpy.shutdown()
    
    
#program starts here
if __name__ == '__main__':
    main()
    