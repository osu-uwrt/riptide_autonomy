#! /usr/bin/env python3

import rclpy
import rclpy.time
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class CompleteBinsAction(Node):
    def __init__(self):
        super().__init__('AlignBins')
        self.actionServer = ActionServer(self, CompleteBins, 'AlignTorpedos', self.completeBinsCallback)
        
    
    
    def completeBinsCallback(self, goalHandle): #ACTION EXECUTED HERE! CALLED ONCE!
        self.get_logger().info("Bins task moment!")
        
        goalHandle.succeed()
        return CompleteBinsResult()


def main(args=None):
    rclpy.init(args=args)
    
    action = CompleteBinsAction()
    action.get_logger().info("Starting Bins action client")
    rclpy.spin(action, executor=MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
