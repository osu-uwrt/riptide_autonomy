#! /usr/bin/env python3

from queue import Empty, Queue
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from riptide_msgs2.action import AlignTorpedos
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from stereo_msgs.msg import DisparityImage
from image_geometry import StereoCameraModel
import utils.TorpedoVision as TorpedoVision
import cv2

# Settings
SHOW_DEBUG_IMAGE = False

# Topic names
ODOM_TOPIC       = "odometry/filtered"
IMG_TOPIC        = "stereo/left/image_raw"
DISPARITY_TOPIC  = "stereo/disparity"
RIGHT_INFO_TOPIC = "stereo/left/camera_info"
LEFT_INFO_TOPIC  = "stereo/right/camera_info"

class AlignTorpedosAction(Node):
    def __init__(self):
        super().__init__("AlignTorpedo")
        self.actionServer = ActionServer(self, AlignTorpedos, 'AlignTorpedos', self.executeCB)
        
        #storage for data received from topics
        self.odomQueue      = Queue(1)
        self.imgQueue       = Queue(1)
        self.disparityQueue = Queue(1)
        self.leftDataQueue  = Queue(1)
        self.rightDataQueue = Queue(1)
        
        self.cvBridge = CvBridge()
        
        self.odomSub  = self.create_subscription(Odometry, ODOM_TOPIC, self.odomCB, qos_profile_system_default)
        self.imgSub   = self.create_subscription(Image, IMG_TOPIC, self.imgCB, qos_profile_system_default)
        self.dispSub  = self.create_subscription(DisparityImage, DISPARITY_TOPIC, self.disparityCB, qos_profile_system_default)
        self.leftSub  = self.create_subscription(CameraInfo, LEFT_INFO_TOPIC, self.leftDataCB, qos_profile_system_default)
        self.rightSub = self.create_subscription(CameraInfo, RIGHT_INFO_TOPIC, self.rightDataCB, qos_profile_system_default)
        
        self.camModel = StereoCameraModel() #this will get configured when the action is called.
        
        self.get_logger().info("Align Torpedos Action Started.")
        
        
    def configureCameraModel(self, timeout = 5.0) -> bool:
        self.get_logger().info("Waiting for camera info")
        try:
            leftInfo = self.leftDataQueue.get(block=True, timeout=timeout)
            rightInfo = self.rightDataQueue.get(block=True, timeout=timeout)
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
    
    
    def imgCB(self, msg):
        cvimg = self.cvBridge.imgmsg_to_cv2(msg)
        self.pushQueue(self.imgQueue, cvimg)
        
    def disparityCB(self, msg):
        self.pushQueue(self.disparityQueue, msg)        
    
    def leftDataCB(self, msg):
        self.pushQueue(self.leftDataQueue, msg)
        
        
    def rightDataCB(self, msg):
        self.pushQueue(self.rightDataQueue, msg)
    
    def executeCB(self, goalHandle):
        self.get_logger().info("Attempting to align the torpedos")
        startTime = self.get_clock().now()
        self.configureCameraModel() #configure camera model with camera info data
        
        goalDist = goalHandle.request.goaldistance
        timeout = goalHandle.request.timeoutms
        
        coords = [] #location of circles in world frame, paired with number of segments of that shape
        
        # collect 25 good detections or time out after the specified timeout time
        while (len(coords) < 50) and (self.get_clock().now() - startTime).to_msg().sec * 1000.0 < timeout:
            try:
                img = self.imgQueue.get(block=True, timeout=1.0)
                disp: DisparityImage = self.disparityQueue.get(block=True, timeout=1.0)
                dispImg = self.cvBridge.imgmsg_to_cv2(disp.image)
                                            
                if img is not None:
                    holes = TorpedoVision.processImage(img) #returns a list of (contour, numCorners)
                    rects = [(cv2.boundingRect(hole), corners) for (hole, corners) in holes]
                                    
                    for ((x, y, w, h), segments) in rects:
                        globalCoords = self.camModel.projectPixelTo3d((x, y), dispImg[x, y])
                        self.get_logger().info("{}".format(globalCoords))
                        coords.append((globalCoords, segments))
                                            
                    for ((x, y, w, h), segments) in rects:
                        cv2.circle(img, (int(x + (w/2)), int(y + (h/2))), 2, (0, 255, 0), 2)
                                        
                    if SHOW_DEBUG_IMAGE:
                        cv2.imshow("result", img)
                        cv2.waitKey(20)
            except Empty: #nothing in queue
                self.get_logger().error("Nothing in image queues! Check that {} and {} are getting published to!".format(IMG_TOPIC, DISPARITY_TOPIC))
        
        goalHandle.succeed()
        return AlignTorpedos.Result()
    

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
