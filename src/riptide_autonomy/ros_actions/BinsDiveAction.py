#! /usr/bin/env python3
#Really calculate dive/align for bins
#Assumes talos down view camera can view bins

from queue import Empty, Queue
import math

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

from utils.BinsVision import processImage


#TODO - find correct image topic
RIGHT_IMAGE_TOPIC = "stereo/right/image_rect_color"
LEFT_IMAGE_TOPIC = "stereo/left/image_rect_color"
ODOM_TOPIC= "/talos/odometry/filtered"

TARGET_DETECTIONS = 50
DETECTIONS_TIMEOUT = 5

DETECTION_GROUP_RADIUS = 20

class lidDetection():
    def __init__(self, x, y, w, h):
        self.x = x
        self.y = y
        self.w = w
        self.h = h

class BinsDiveAction(Node):
    def __init__(self):
        super().__init__('AlignBins')
        self.actionServer = ActionServer(self, BinsDive, 'AlignTorpedos', self.BinsDiveCallback)
        
        #subs
        self.odomSub = self.create_subscription(Odometry, ODOM_TOPIC, self.odomCallback, qos_profile_system_default)
        self.leftImageSub = self.create_subscription(Image, LEFT_IMAGE_TOPIC, self.leftImageCallback, qos_profile_system_default)
        self.rightImageSub = self.create_subscription(Image, RIGHT_IMAGE_TOPIC, self.rightImageCallback, qos_profile_system_default)

        #queues for sub data
        self.odomQueue = Queue(1)
        self.rightImageQueue = Queue(1)
        self.leftImageQueue = Queue(1)
    

    def dist(pt1, pt2) -> float:
        return math.sqrt(math.pow(pt1[0] - pt2[0], 2) + math.pow(pt1[1] - pt2[1], 2))

    def locationAverage(locations: "list[lidDetection]"):
        sumX = 0
        sumY = 0

        for location in locations:
            sumX = sumX + location.x
            sumY = sumY + location.y

            averageLocation = {sumX / len(locations), sumY / len(locations)}

        return averageLocation

    def odomCallback(self, msg):
        if self.odomQueue.full():
            self.odomQueue.get_nowait()
            #if full pull out msg

        self.odomQueue.put_nowait(msg)

    def obtainCurrentOdometry(self, timeout=3.0) -> Odometry:
        #get from queue
        return self.odomQueue.get()

    def leftImageCallback(self, msg):
        if self.leftImageQueue.full():
            self.leftImageQueue.get_nowait()

        self.leftImageQueue.put_nowait(msg)   
    
    def rightImageCallback(self, msg):
        if self.rightImageQueue.full():
            self.rightImageQueue.get_nowait()

        self.rightImageQueue.put_nowait(msg)

    def collectDetection(self, side, timeout=1.0) -> lidDetection:
        #set side to true for left detection, false for right detection
        try:
            if side:
                image = self.leftImageQueue.get(block=True, timeout=timeout)
            else:
                image = self.rightImageQueue.get(block=True, timeout=timeout)

            lidRect, binAngle = processImage(image) # returns a rectangle and an angle

            if(lidRect[0] == -1):
                self.get_logger().warn("Cannot detect bins!! Getting Image")
                return None

            return lidDetection(lidRect[0], lidRect[1], lidRect[2], lidRect[3])

        except:
            self.get_logger().error("Image Queue Empty in Bins!! Subbing to {}", LEFT_IMAGE_TOPIC)

        return None

    def groupDetections(self, detections: "list[lidDetection]"):
        groups: list[list[lidDetection]] = []

        for detection in detections:
            grouped = False
            location = (detection.x, detection.y)

            for group in groups:
                groupLocation = self.locationAverage(group)
                
                if self.dist(groupLocation, location) < DETECTION_GROUP_RADIUS:
                    if grouped == False:
                        grouped = True

                        group.append(detection)
            
            if grouped == False:
                groups.append([detections])

        if len(group) == 0:
            self.get_logger().warn("Bins Alignment: Groups Empty")

        return groups

    
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
        #TODO - find real numbers - foot? starboard foot?

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


def excuteAlginment(self):
    self.get_logger().info("Starting Bins Fine Alginment")

    leftDetections = []
    rightDetections = []

    #collect detectoins until target reached or until timeout
    collectionStartTime = self.get_clock().now()
    while (len(leftDetections) < TARGET_DETECTIONS or len(rightDetections) < TARGET_DETECTIONS) and (collectionStartTime + DETECTIONS_TIMEOUT > self.get_clock().now()):
        leftLidRect = self.collectDetection(self, True)
        leftDetections.append(leftLidRect)

        rightLidRect = self.collectDetection(self, False)
        rightDetections.append(rightLidRect)

    if leftDetections < TARGET_DETECTIONS or rightDetections < TARGET_DETECTIONS:
        self.get_logger().warn("Did not reach target detections. Target: {}, Dectections: {}", TARGET_DETECTIONS, len(rightDetections), len(leftDetections))

    leftGroups = self.groupDetections(leftDetections)
    rightGroups = self.groupDetections(rightDetections)

    #determine the correct detection to use
    filteredDetections: list[lidDetection] = []

def main(args=None):
    rclpy.init(args=args)
    
    action = BinsDiveAction()
    action.get_logger().info("Starting Bins action client")
    rclpy.spin(action, executor=MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
