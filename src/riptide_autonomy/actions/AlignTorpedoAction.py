#! /usr/bin/env python3

from math import cos, sin
from queue import Queue

import cv2
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

#TODO: figure out these values
PROP_SIZE_x = 10 #meters
PROP_SIZE_y = 10 #also meters

ROBOT_NAME = "tempest"

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
    def imageToGlobalCoordinates(self, x, y, distance, pixPerM):
        xMeters = x / pixPerM
        yMeters = y / pixPerM
        
        try:
            toFrame = "world"
            fromFrame = ROBOT_NAME + "/stereo/left_link"
            now = self.get_clock().now()
            
            cameraToWorld = self.tfBuffer.lookup_transform(
                toFrame,
                fromFrame,
                now
            )
            
            globalCoords = self.transformPosition((x, y, distance), cameraToWorld)
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
        
    
    def waitForImage(self, timeout=3.0):
        return self.imgQueue.get(True, timeout)
        
        
    def alignTorpedosCallback(self, goalHandle):
        self.get_logger().info("Starting align torpedos action.")
        distance = goalHandle.request.distance
        torpedo = goalHandle.request.torpedo
        timeout = goalHandle.request.timeoutms
        
        startTime = self.get_clock().now()
        
        #TODO: make TorpedoVision.processImage() return the coordinates of the star and circle respectively (you can probably narrow down a lot by checking the top-leftmost and bottom-rightmost contours within the prop itself), then that makes our job in this loop much much eaiser
        imageCoords = []
        
        # collect 25 good detections or time out after the specified timeout time
        while len(imageCoords) < 25 and (self.get_clock().now() - startTime).to_msg().sec * 1000.0 < timeout:
            img = self.waitForImage()
                        
            if img is not None:                
                holes = TorpedoVision.processImage(img)
                                
                for [x, y, w, h] in holes:
                    cv2.circle(img, (int(x + (w/2)), int(y + (h/2))), 2, (0, 255, 0), 2)
                    
                cv2.imshow("result", img)
                cv2.waitKey(1)
        
                
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
    