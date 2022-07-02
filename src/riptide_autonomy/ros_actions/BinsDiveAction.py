#! /usr/bin/env python3
#Really calculate dive/align for bins
#Assumes Tempest down view camera can view bins

from queue import Queue

import rclpy
import rclpy.time
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from geometry_msgs.msg import Vector3, Pose
from riptide_msgs2.action import BinsDive
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

from ros_actions import doTransform


#TODO - find correct image topic
IMAGE_TOPIC = "/tempest/stereo/down/image_rect_color"
ODOM_TOPIC= "/tempest/odometry/filtered"

class BinsDiveAction(Node):
    def __init__(self):
        super().__init__('AlignBins')
        self.actionServer = ActionServer(self, BinsDive, 'AlignTorpedos', self.BinsDiveCallback)
        
        #subs
        self.odomSub = self.create_subscription(Odometry, ODOM_TOPIC, self.odomCallback, qos_profile_system_default)
        self.imageSub = self.create_subscription(Image, IMAGE_TOPIC, self.imageCallback, qos_profile_system_default)

        #queues for sub data
        self.odomQueue = Queue(1)
        self.imageQueue = Queue(1)
    

    
    def odomCallback(self, msg):
        if self.odomQueue.full():
            self.odomQueue.get_nowait()
            #if full pull out msg

        self.odomQueue.put_nowait(msg)

    def obtainCurrentOdometry(self, timeout=3.0) -> Odometry:
        #get from queue
        return self.odomQueue.get()

    def imageCallback(self, msg):
        if self.imageQueue.full():
            self.imageQueue.get_nowait()

        self.odomQueue.put_nowait(msg)

    def obtainCurrentImage(self, timeout=3.0) -> Odometry:
        return self.imageQueue.get()
    
    def BinsDiveCallback(self, goalHandle): #ACTION EXECUTED HERE! CALLED ONCE!
        self.get_logger().info("Bins task moment!")

        actionResult = BinsDive.Result()

        #find target coordinates/orentation -- cough cough pose
        handlePosition = Vector3() # not to be confused with target position
        relativeTargetPose = Pose()

        #TODO - base these off of vision data
        handlePosition.x = 3
        handlePosition.y = 4
        handlePosition.z = 5

        relativeTargetPose.orientation.z = 30 # yaw

        #Target Pitch and Roll will be assume to be zero
        relativeTargetPose.orientation.x = 0
        relativeTargetPose.orientation.y = 0



        #account for offset required for the claw
        #TODO - find real numbers - is claw mounted yet? is there 3D model for reference
        positionOffset = Vector3()
        positionOffset.x = 2
        positionOffset.y = 3
        positionOffset.z = 4

        relativeTargetPose.position.x = handlePosition.x - positionOffset.x
        relativeTargetPose.position.y = handlePosition.y - positionOffset.y
        relativeTargetPose.position.z = handlePosition.z - positionOffset.z
        
        #transform coordinates to absolute

        actionResult.pose = doTransform(relativeTargetPose,)

        goalHandle.succeed()
        return actionResult


def main(args=None):
    rclpy.init(args=args)
    
    action = BinsDiveAction()
    action.get_logger().info("Starting Bins action client")
    rclpy.spin(action, executor=MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
