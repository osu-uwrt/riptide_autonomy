#! /usr/bin/env python3

from queue import Queue
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
from image_geometry import StereoCameraModel

ODOM_TOPIC = "odometry/filtered"
IMG_TOPIC  = "stereo/left/image_raw"
RIGHT_INFO_TOPIC = "stereo/left/camera_info"
LEFT_INFO_TOPIC  = "stereo/right/camera_info"

class AlignTorpedosAction(Node):
    def __init__(self):
        super().__init__("AlignTorpedo")
        self.actionServer = ActionServer(self, AlignTorpedos, 'AlignTorpedos', self.executeCB)
        
        #storage for data received from topics
        self.odomQueue = Queue(1)
        self.imgQueue  = Queue(1)
        self.leftDataQueue = Queue(1)
        self.rightDataQueue = Queue(1)
        
        self.odomSub  = self.create_subscription(Odometry, ODOM_TOPIC, self.odomCB, qos_profile_system_default)
        self.imgSub   = self.create_subscription(Image, IMG_TOPIC, self.imgCB, qos_profile_system_default)
        self.leftSub  = self.create_subscription(CameraInfo, LEFT_INFO_TOPIC, self.leftDataCB, qos_profile_system_default)
        self.rightSub = self.create_subscription(CameraInfo, RIGHT_INFO_TOPIC, self.rightDataCB, qos_profile_system_default)
        
        self.camModel = StereoCameraModel()
        self.configureCameraModel()
        
        self.get_logger().info("Align Torpedos Action Started.")
        
        
    def configureCameraModel(self):
        self.get_logger().info("Waiting for camera info")
        leftInfo = self.leftDataQueue.get(block=True)
        rightInfo = self.rightDataQueue.get(block=True)
        self.camModel.fromCameraInfo(leftInfo, rightInfo)
        self.get_logger().info("Camera info received")
    
    
    def pushQueue(self, q: Queue, obj):
        if not q.empty():
            q.get_nowait()
        
        q.put_nowait(obj)
    
    
    def odomCB(self, msg):
        self.pushQueue(self.odomQueue, msg)
    
    
    def imgCB(self, msg):
        cvimg = CvBridge.imgmsg_to_cv2(msg)
        self.pushQueue(self.imgQueue, cvimg)
        
    
    def leftDataCB(self, msg):
        self.pushQueue(self.leftDataQueue, msg)
        
        
    def rightDataCB(self, msg):
        self.pushQueue(self.rightDataQueue, msg)

    
    def executeCB(self, goalHandle):
        self.get_logger().info("Attempting to align the torpedos")
        
        #to be continued
        
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
