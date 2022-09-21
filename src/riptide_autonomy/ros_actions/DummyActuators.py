#! /usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from riptide_msgs2.action import ActuateDroppers as DropperMsg

class DummyActuatorServers(Node):

    def __init__(self):
        super().__init__('dummy_actuator_servers')
        
        # self.torpedo_server = ActionServer(
        #     self,
        #     TorpedoMsg,
        #     'torpedo',
        #     self.torpedo_callback)

        self.dropper_server = ActionServer(
            self,
            DropperMsg,
            'dropper',
            self.dropper_callback)
        
        # self.claw_server = ActionServer(
        #     self,
        #     ClawMsg,
        #     'claw',
        #     self.claw_callback)
        
        self.get_logger().info("Dummy actuator server started.")

    # def torpedo_callback(self, goal_handle):
    #     torpedoNum = 0 #TODO
    #     self.get_logger().info("Firing Torpedo {}".format(torpedoNum))
    #     goal_handle.succeed()
    #     return TorpedoMsg.Result()

    def dropper_callback(self, goal_handle):
        dropperNum = goal_handle.request.dropper_id
        self.get_logger().info("Dropping dropper {}".format(dropperNum))
        goal_handle.succeed()
        return DropperMsg.Result()
    
    # def claw_callback(self, goal_handle):
    #     open_claw = False #TODO
    #     self.get_logger().info("Opening Claw" if open_claw else "Closing Claw")
    #     goal_handle.succeed()
    #     return ClawMsg.Result()


def main(args=None):
    rclpy.init(args=args)
    dummy_actuator_servers = DummyActuatorServers()
    rclpy.spin(dummy_actuator_servers)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Dummy actuator server keyboard-interrupted.")
        pass
