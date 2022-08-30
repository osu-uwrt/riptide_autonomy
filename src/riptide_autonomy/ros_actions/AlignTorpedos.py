#! /usr/bin/env python3

from __future__ import annotations

from math import cos, inf, sin, sqrt
from queue import Empty, Queue

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped, Vector3
from image_geometry import StereoCameraModel
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from riptide_msgs2.action import AlignTorpedos
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion

import utils.TorpedoVision as TorpedoVision

# Settings
DEBUG_MODE                          = True
DEBUG_OUTPUT_TOPIC                  = "align_torpedos_debug"
DEBUG_RATE                          = 0.125
NUM_DETECTIONS                      = 50 #number of detections to gather for processing.
DETECTION_GROUP_RADIUS              = 25
NON_CIRCLE_PRIORITY                 = 1.3 #measure of how much more "important" non-circle shapes are. this will affect how the alignment prioritizes the special shapes for more points
MAX_LEFT_RIGHT_DETECTION_DIFFERENCE = 10 #maximum number of times a detection is allowed to be seen by one camera but not the other

#frame names
ROBOT_NAME        = "tempest"
TORPEDO_FRAME     = ROBOT_NAME + "/torpedo_link"
BASE_LINK_FRAME   = ROBOT_NAME + "/base_link"
LEFT_CAMERA_FRAME = ROBOT_NAME + "/stereo/left_link"
PROP_FRAME        = "torpedoGman_frame"

# Topic names
ODOM_TOPIC       = "odometry/filtered"
LEFT_IMG_TOPIC   = "stereo/left_raw/image_raw_color"
# LEFT_IMG_TOPIC  = "stereo/left/image_raw"
RIGHT_IMG_TOPIC  = "stereo/right/image_rect_color"
DISPARITY_TOPIC  = "stereo/disparity"
RIGHT_INFO_TOPIC = "stereo/right_info"
LEFT_INFO_TOPIC  = "stereo/left_info"

class HoleDetection:
    def __init__(self, x, y, w, h, numSegments): #format of rects is ((x, y, w, h), numSegments)
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.numSegments = numSegments
        
        
def avgDetection(detections: list[HoleDetection]) -> HoleDetection:
    xAvg = 0
    yAvg = 0
    wAvg = 0
    hAvg = 0
    nAvg = 0 #segment count
    detCount = len(detections)
    
    for detection in detections:
        xAvg += detection.x / detCount
        yAvg += detection.y / detCount
        wAvg += detection.w / detCount
        hAvg += detection.h / detCount
        nAvg += detection.numSegments / detCount

    return HoleDetection(xAvg, yAvg, wAvg, hAvg, nAvg)


def groupDetections(detections: list[HoleDetection]) -> list[list[HoleDetection]]:
    #group detections together by location and shape
    groups: list[list[HoleDetection]] = [] #will be a list of (groups of coordinates that are close together, numSegments)
    for detection in detections:
        detLocation = (detection.x, detection.y)
        
        #find all groups that the coordinate belongs to
        foundGroup = False
        for i in range(0, len(groups)):
            avgDet = avgDetection(groups[i])
            groupLocation = (avgDet.x, avgDet.y)
            
            if TorpedoVision.dist(detLocation, groupLocation) < DETECTION_GROUP_RADIUS and detection.numSegments == groups[i][0].numSegments:
                groups[i].append(detection)
                foundGroup = True

        if not foundGroup:
            groups.append([detection])
    
    return groups

def rotateAboutYaw(pt: Vector3, angle) -> Vector3:
    x = pt.x * cos(angle) - pt.y * sin(angle)
    y = pt.x * sin(angle) + pt.y * cos(angle)
    
    res = Vector3()
    res.x = x
    res.y = y
    res.z = pt.z
    
    return res

#basically this method should be the exact python version of doTransform() in util.cpp but without transforming the quaternion and returning a pose
def doTransform(coords: Vector3, transform: TransformStamped) -> Vector3:
    #rotate point with transform orientation
    rpy = euler_from_quaternion([
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w
    ])
    
    yaw = rpy[2]
    relative = rotateAboutYaw(coords, yaw)
    
    position = Vector3()
    position.x = relative.x + transform.transform.translation.x
    position.y = relative.y + transform.transform.translation.y
    position.z = relative.z + transform.transform.translation.z
    
    return position

            

class AlignTorpedosAction(Node):
    def __init__(self):
        super().__init__("AlignTorpedo")
        self.actionServer = ActionServer(self, AlignTorpedos, 'AlignTorpedos', self.executeCB)
        
        #TF stuff
        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer, self)
        
        #storage for data received from topics
        self.odomQueue      = Queue(1)
        self.leftImgQueue   = Queue(1)
        self.rightImgQueue  = Queue(1)
        self.leftInfoQueue  = Queue(1)
        self.rightInfoQueue = Queue(1)
        
        self.cvBridge = CvBridge()
        
        self.odomSub     = self.create_subscription(Odometry, ODOM_TOPIC, self.odomCB, qos_profile_system_default)
        self.leftImgSub  = self.create_subscription(Image, LEFT_IMG_TOPIC, self.leftImgCB, qos_profile_system_default)
        self.rightImgSub = self.create_subscription(Image, RIGHT_IMG_TOPIC, self.rightImgCB, qos_profile_system_default)
        self.leftSub     = self.create_subscription(CameraInfo, LEFT_INFO_TOPIC, self.leftDataCB, qos_profile_system_default)
        self.rightSub    = self.create_subscription(CameraInfo, RIGHT_INFO_TOPIC, self.rightDataCB, qos_profile_system_default)
        
        self.camModel    = StereoCameraModel() #this will get configured when the action is called.
        
        if DEBUG_MODE:
            self.timer = self.create_timer(DEBUG_RATE, self.debugCB)
            self.outPub = self.create_publisher(Image, DEBUG_OUTPUT_TOPIC, qos_profile_system_default)
        
        self.get_logger().info("Align Torpedos Action Started.")
        
        
    def configureCameraModel(self, timeout = 5.0) -> bool:
        self.get_logger().info("Waiting for camera info")
        try:
            leftInfo = self.leftInfoQueue.get(block=True, timeout=timeout)
            rightInfo = self.rightInfoQueue.get(block=True, timeout=timeout)
            self.camModel.fromCameraInfo(leftInfo, rightInfo)
            self.get_logger().info("Camera info received")
            return True
        except Empty:
            self.get_logger().error("No camera info was available.")
            return False
    
    
    def pushQueue(self, q: Queue, obj):
        if not q.empty():
            q.get_nowait()
        
        q.put_nowait(obj)
    
    
    def odomCB(self, msg):
        self.pushQueue(self.odomQueue, msg)
    
    def leftImgCB(self, msg):
        cvimg = self.cvBridge.imgmsg_to_cv2(msg)
        self.pushQueue(self.leftImgQueue, cvimg)
    
    def rightImgCB(self, msg):
        cvimg = self.cvBridge.imgmsg_to_cv2(msg)
        self.pushQueue(self.rightImgQueue, cvimg)
    
    def leftDataCB(self, msg):
        self.pushQueue(self.leftInfoQueue, msg)
        
    def rightDataCB(self, msg):
        self.pushQueue(self.rightInfoQueue, msg)
        
    def transformBetweenFrames(self, pose: Vector3, fromFrame: str, toFrame: str, timeout=5.0) -> Vector3:
        startTime = self.get_clock().now()
        #attempt to look up transform until lookup succeeds or timeout is reached (or rclpy is shut down)
        while rclpy.ok() and (self.get_clock().now() - startTime).to_msg().sec < timeout:
            try:
                transform = self.tfBuffer.lookup_transform(
                    toFrame,
                    fromFrame,
                    rclpy.time.Time()
                )
                                
                return doTransform(pose, transform)
            except Exception as ex:
                self.get_logger().warn("Received exception while looking up transform: {}".format(ex))
        
        self.get_logger().error("Failed to look up transform from {} to {}!".format(fromFrame, toFrame))
        
        return None
        
    def collectDetections(self, imgQueue: Queue, detections: list[HoleDetection]):
        try:
            img = imgQueue.get(block=True, timeout=1.0)
                                        
            holes, _ = TorpedoVision.processImage(img) #returns a list of (contour, numCorners)
            
            rects = []
            for (hole, segments) in holes:
                (x, y, w, h) = cv2.boundingRect(hole)
                cenX = x + (w / 2)
                cenY = y + (h / 2)
                rects.append((cenX, cenY, w, h))
                detections.append(HoleDetection(cenX, cenY, w, h, segments))
                                
            return img, rects
        except Empty: #nothing in queue
            self.get_logger().error("Empty image queue! Check that {} and {} are getting published to!".format(LEFT_IMG_TOPIC, RIGHT_IMG_TOPIC))
            return None, None
    
    
    def executeCB(self, goalHandle):
        self.get_logger().info("Attempting to align the torpedos")
        startTime = self.get_clock().now()
        self.configureCameraModel() #configure camera model with camera info data
        
        goalDist = goalHandle.request.goaldistance
        timeout = goalHandle.request.timeoutms
        
        leftDetections = [] #left camera detections
        rightDetections = [] #right camera detections
        
        # collect 25 good detections or time out after the specified timeout time
        while (len(leftDetections) < NUM_DETECTIONS) and (len(rightDetections) < NUM_DETECTIONS) and (self.get_clock().now() - startTime).to_msg().sec * 1000.0 < timeout:
            leftImg, leftRects = self.collectDetections(self.leftImgQueue, leftDetections)
            # rightImg, rightRects = self.collectDetections(self.rightImgQueue, rightDetections)
                                
            if DEBUG_MODE:
                for (x, y, w, h) in leftRects:
                    cv2.circle(leftImg, (int(x + (w/2)), int(y + (h/2))), 2, (0, 255, 0), 2)
                
                # for (x, y, w, h) in rightRects:
                #     cv2.circle(rightImg, (int(x + (w/2)), int(y + (h/2))), 2, (0, 255, 0), 2)
                
        if len(leftDetections) < NUM_DETECTIONS:
            self.get_logger().warn("Timed out before all detections could be gathered! Collected {} left detections.".format(len(leftDetections)))
        
        leftGroups  = groupDetections(leftDetections)        
        
        #choose detection to align to
        scores = []
        for group in leftGroups:
            multiplier = NON_CIRCLE_PRIORITY if group[0].numSegments != 8 else 1           
            scores.append(len(group) * multiplier)
        
        if len(scores) > 0:
            maxIndex = scores.index(max(scores)) #index of max of scores
            target = avgDetection(leftGroups[maxIndex]) #max score means most valueable target
            
            #get distance to target (used to calculate disparity)
            odom: Odometry = self.odomQueue.get(block=True, timeout=2.5)
            curPos: Vector3 = odom.pose.pose.position
            propPos: Vector3 = self.transformBetweenFrames(Vector3(), PROP_FRAME, BASE_LINK_FRAME)
            dist = abs(curPos.x - propPos.x)
            targetDisparity = self.camModel.getDisparity(dist)
            
            #get 3d position of hole in camera frame
            holeCamPos = Vector3()
            (holeCamPos.z, holeCamPos.y, holeCamPos.x) = self.camModel.projectPixelTo3d((target.x, target.y), targetDisparity) # In camera frame: (z, y, x)
            # (holeCamPos.x, holeCamPos.y, holeCamPos.z) = self.camModel.projectPixelTo3d((target.x, target.y), targetDisparity) # In camera frame: (z, y, x)
            self.get_logger().info("Found hole at camera frame coords {}, {}, {} with {} segments".format(holeCamPos.x, holeCamPos.y, holeCamPos.z, target.numSegments))
            
            #figure out where to move Tempest to have torpedos aligned with the hole
            torpedoPosition = self.transformBetweenFrames(Vector3(), TORPEDO_FRAME, BASE_LINK_FRAME) #position of base link relative to torpedo launcher
            holeTorpedoPos = self.transformBetweenFrames(holeCamPos, LEFT_CAMERA_FRAME, TORPEDO_FRAME) #location of detection in torpedo frame
            
            if torpedoPosition is None or holeTorpedoPos is None:
                self.get_logger().error("Failed to perform some necessary transforms!")
                goalHandle.abort()
                return AlignTorpedos.Result()           
            
            holeTorpedoPos.x -= goalDist
            
            goalBasePos = self.transformBetweenFrames(holeTorpedoPos, TORPEDO_FRAME, "world")
            
            if goalBasePos is None:
                self.get_logger().error("Failed to perform some necessary transforms!")
                goalHandle.abort()
                return AlignTorpedos.Result()
            
            #offset torpedo position to get goal position
            goalPosition = Vector3()
            goalPosition.x = goalBasePos.x - torpedoPosition.x
            goalPosition.y = goalBasePos.y - torpedoPosition.y
            goalPosition.z = goalBasePos.z - torpedoPosition.z

            actionResult = AlignTorpedos.Result()
            actionResult.coords = goalPosition
                        
            goalHandle.succeed()
            self.get_logger().info("Torpedo Alignment Calculation Complete.")
            return actionResult

        self.get_logger().error("No attemptable targets found! Aborting!")
        goalHandle.abort()
        return AlignTorpedos.Result()

    def debugCB(self):
        if DEBUG_MODE:
            try:
                img = self.leftImgQueue.get(block=True, timeout=DEBUG_RATE / 2)
                _, outImg = TorpedoVision.processImage(img)
                
                msg = self.cvBridge.cv2_to_imgmsg(outImg, encoding='rgb8')
                self.outPub.publish(msg)
            except Empty:
                pass
        else:
            self.get_logger().warn("Enable DEBUG_MODE to debug!")
    

def main(args=None):
    rclpy.init(args=args)
    act = AlignTorpedosAction()
    rclpy.spin(act, executor=MultiThreadedExecutor())
    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Align Torpedos Action Keyboard Interrupted")
