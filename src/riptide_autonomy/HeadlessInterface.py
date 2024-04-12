#! /usr/bin/env python3

import os
import math
import rclpy
from rclpy.node import Node
from ament_index_python import get_package_share_directory
from nortek_dvl_msgs.msg import DvlStatus
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from std_msgs.msg._bool import Bool
from chameleon_tf_msgs.action import ModelFrame
from riptide_msgs2.action import ExecuteTree
from riptide_msgs2.msg import LedCommand
from transforms3d.euler import quat2euler
from rclpy.qos import qos_profile_sensor_data
from enum import Enum

#To run this program:
#ros2 run riptide_autonomy2 StateMachine.py

#CODE IS DONE
#JUST CHECK IF IT DOES WHAT IT NEEDS TO DO
#ros2 topic pub -r8 /dvl/status nortek_dvl_msgs/msg/DvlStatus
#ros2 topic pub -r8 /odometry/filtered nav_msgs/msg/Odometry
#ros2 topic pub -r8 /state/aux std_msgs.msg/_bool/Bool
#ros2 run remote_launch launcher, go to localhost:8080 then .yaml
#run mapping and autonomy
#ros2 launch riptide_autonomy2 autonomy.launch.py
#ros2 launch riptide_mapping2 mapping.launch.py

#Run these 3 in order. Get DvlStatus which will run Odometry checks if good, then start tag cal then start behavior tree.

#Project description
#The first thing this code does is subscribe to a topic that gets DvlStatus data.
#The code checks if the data from DvlStatus is good; if it is, will then start a subscriber node to Odometry.
#The subscription to Odometry is to check if the robot's movement is jittery or flying off to infinity.
#Once odometry data is good, start a tag cal and finally the behavior tree.

class RobotStateId(Enum):
    NO_STATE = -1
    WAITING_FOR_DVL = 0
    WAITING_FOR_GOOD_ODOM = 1
    WAITING_FOR_KILL = 2
    WAITING_FOR_AUX = 3
    PERFORMING_TAG_CAL = 4
    WAITING_FOR_TREE = 5
    TREE_STARTED = 6

class RobotState:
    def __init__(self, id: RobotStateId, led_cmd: LedCommand, entrance_function = None):
        self.id = id
        self.led_cmd = led_cmd
        self.entrance_function = entrance_function
        
        # if True, the machine will move on to the next state
        self.passed = False 
    
    # called when a state is entered
    def enter(self, node: Node):
        self.passed = False
        node.get_logger().info(f"Entering state {self.id.name}")
        if self.entrance_function is not None:
            self.entrance_function()

#If roll or pitch pass this limit, output an error
ROLL_LIMIT = 30 #degrees
PITCH_LIMIT = 30 #degrees
ACCEL_LIMIT = 1
class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')
        
        # behavior tree file to run
        self.declare_parameter("behaviortree_to_run", "")
        
        # dict of possible states, keys are state ids
        self.states: dict[RobotStateId, RobotState] = {}
        
        self.update_timer = self.create_timer(1, self.update_state)
        self.ledPub = self.create_publisher(LedCommand, "command/led", 10)

        self.subscription_dvl = self.create_subscription(
            DvlStatus,
            'dvl/status',
            self.dvl_callback,
            qos_profile_sensor_data)

        self.subscription_odom = self.create_subscription(
            Odometry, #Class
            'odometry/filtered', #This is topic name
            self.odometry_callback,
            10)
        
        self.subscription_kill = self.create_subscription(
            Bool, #Class
            'state/kill', #This is topic name
            self.kill_callback,
            10)

        self.subscription_aux = self.create_subscription(
            Bool, #Class
            'state/aux', #This is topic name
            self.aux_callback,
            10)
        
        self.tag_client = ActionClient(
            self,
            ModelFrame,
            'map/model_tf')
        
        self.autonomy_client = ActionClient(
            self, 
            ExecuteTree,
            'autonomy/run_tree')
        
        # register states here
        self.register_state(RobotStateId.WAITING_FOR_DVL, 255, 255, 255, LedCommand.MODE_BREATH, entrance_function=self.invalidate_tag_cal)
        self.register_state(RobotStateId.WAITING_FOR_GOOD_ODOM, 255, 0, 0, LedCommand.MODE_FAST_FLASH, entrance_function=self.invalidate_tag_cal)
        self.register_state(RobotStateId.WAITING_FOR_KILL, 255, 0, 0, LedCommand.MODE_SLOW_FLASH, entrance_function=self.invalidate_tag_cal)
        # self.register_state(RobotStateId.WAITING_FOR_TAG, 255, 0, 0, LedCommand.MODE_BREATH)
        self.register_state(RobotStateId.WAITING_FOR_AUX, 0, 0, 255, LedCommand.MODE_BREATH)
        self.register_state(RobotStateId.PERFORMING_TAG_CAL, 255, 255, 255, LedCommand.MODE_FAST_FLASH, entrance_function=self.send_goal_tag)
        self.register_state(RobotStateId.WAITING_FOR_TREE, 255, 100, 0, LedCommand.MODE_FAST_FLASH)
        self.register_state(RobotStateId.TREE_STARTED, 0, 255, 0, LedCommand.MODE_FAST_FLASH)
        
        # initial state
        self.current_state = RobotStateId.NO_STATE
        
        # keeping track of odometry
        self.last_odom = Odometry()
        
        #circular buffer of dvl status messages
        self.prev_dvl_statuses = [[False, False, False, False]] * 8
        self.oldest_dvl_status = 0
        

    def register_state(self, id: RobotStateId, led_r: int, led_g: int, led_b: int, 
                       led_mode: int, entrance_function = None):
        led_cmd = LedCommand()
        led_cmd.red = led_r
        led_cmd.green = led_g
        led_cmd.blue = led_b
        led_cmd.mode = led_mode
        
        new_state = RobotState(id, led_cmd, entrance_function)
        self.states[id] = new_state
    

    def at_least_three_true(self, arr):
        count = 0
        for value in arr:
            if value:
                count += 1
        
        return count >= 3
    
    
    def publish_led_status(self, status: RobotState):
        msg = self.states[status].led_cmd
        msg.target = LedCommand.TARGET_ALL
        
        self.ledPub.publish(msg)
        
    
    def update_state(self):
        new_current_state_id = RobotStateId.WAITING_FOR_DVL
        
        #find next state that hasnt passed
        for state_id in RobotStateId:
            new_current_state_id = state_id
            if new_current_state_id in self.states and not self.states[new_current_state_id].passed:
                break
        
        entering_state = new_current_state_id != self.current_state
        self.current_state = new_current_state_id
        
        if entering_state:
            # just entered this state, so call enter function and update leds
            self.publish_led_status(new_current_state_id)
            self.states[new_current_state_id].enter(self)
    

    def go_to_state(self, new_state: RobotStateId, override_tree: bool = False):
        if self.current_state != RobotStateId.TREE_STARTED or override_tree:
            self.states[new_state].passed = False
            self.update_state()
    
    
    def invalidate_tag_cal(self):
        self.states[RobotStateId.WAITING_FOR_AUX].passed = False
        self.states[RobotStateId.PERFORMING_TAG_CAL].passed = False
        self.states[RobotStateId.WAITING_FOR_TREE].passed = False
            

    #FORMAT --> ros2 topic pub -r8 <topic_name> <msg_type>
    #typehinting msg:DvlStatus makes sure msg is of type DvlStats
    def dvl_callback(self, msg: DvlStatus):
        statusMessages = [msg.b1_vel_valid, msg.b2_vel_valid, msg.b3_vel_valid, msg.b4_vel_valid]
        self.prev_dvl_statuses[self.oldest_dvl_status] = statusMessages
        
        self.oldest_dvl_status += 1
        if self.oldest_dvl_status >= len(self.prev_dvl_statuses):
            self.oldest_dvl_status = 0
        
        dvl_good = True
        for i in range(0, len(self.prev_dvl_statuses)):
            if not self.at_least_three_true(self.prev_dvl_statuses[i]):
                dvl_good = False
                break
        
        if dvl_good:
            self.states[RobotStateId.WAITING_FOR_DVL].passed = True
        else:
            self.go_to_state(RobotStateId.WAITING_FOR_DVL)

       
    def odometry_callback(self, msg: Odometry):
        linearVelocityX1 = msg.twist.twist.linear.x
        linearVelocityY1 = msg.twist.twist.linear.y
        linearVelocityZ1 = msg.twist.twist.linear.z
        
        msg_stamp_sec = msg.header.stamp.sec + float(msg.header.stamp.nanosec / 1e9)
        last_odom_stamp_sec = self.last_odom.header.stamp.sec + float(self.last_odom.header.stamp.nanosec / 1e9)
        
        timeDifference = msg_stamp_sec - last_odom_stamp_sec
        linearAccelerationX = (linearVelocityX1 - self.last_odom.twist.twist.linear.x) / timeDifference
        linearAccelerationY = (linearVelocityY1 - self.last_odom.twist.twist.linear.y) / timeDifference
        linearAccelerationZ = (linearVelocityZ1 - self.last_odom.twist.twist.linear.z) / timeDifference

        acceleration = math.sqrt(linearAccelerationX ** 2 + linearAccelerationY ** 2 + linearAccelerationZ ** 2)
        roll, pitch, _ = quat2euler((msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                                     msg.pose.pose.orientation.y, msg.pose.pose.orientation.z))
        
        roll = roll * 180 / math.pi
        pitch = pitch * 180 / math.pi
        
        odom_good = acceleration < ACCEL_LIMIT and roll < ROLL_LIMIT and pitch < PITCH_LIMIT
        
        if odom_good:
            self.states[RobotStateId.WAITING_FOR_GOOD_ODOM].passed = True
        else:
            self.go_to_state(RobotStateId.WAITING_FOR_GOOD_ODOM)
        
        if not odom_good:
            self.get_logger().error(f"Got bad odom with acceleration {acceleration}, roll {roll}, and pitch {pitch}", throttle_duration_sec=1)
        
        self.last_odom = msg


    def send_goal_tree(self):
        goal_msg = ExecuteTree.Goal()
        goal_msg.tree = os.path.join(get_package_share_directory("riptide_autonomy2"), "trees", 
                                     self.get_parameter("behaviortree_to_run").value)
        self.get_logger().info(f"Attempting to run tree {goal_msg.tree}")
        self._send_goal_future = self.autonomy_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback_tree)
    
    
    def goal_response_callback_tree(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback_tree)
        
        self.states[RobotStateId.WAITING_FOR_TREE].passed = True


    def get_result_callback_tree(self, future):
        self.go_to_state(RobotStateId.WAITING_FOR_AUX)
        

    def send_goal_tag(self):
        goal_msg = ModelFrame.Goal()
        goal_msg.monitor_child = "estimated_origin_frame"
        goal_msg.monitor_parent = "world"
        goal_msg.samples = 10

        if not self.tag_client.wait_for_server(3):
            self.get_logger().info("Timed out waiting for tag cal server.")
            self.go_to_state(RobotStateId.WAITING_FOR_AUX)

        self._send_goal_future = self.tag_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback_tag)
        
    
    def goal_response_callback_tag(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            
            # go back to waiting for aux state
            self.go_to_state(RobotStateId.WAITING_FOR_AUX)
            return
        
        self.get_logger().info('Goal accepted :)')
        
        # variable created by send_goal_tag
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback_tag)

    def get_result_callback_tag(self, future):
        result = future.result().result
        if result.success:
            # go to next state
            self.states[RobotStateId.PERFORMING_TAG_CAL].passed = True
        else:
            # wait for aux again
            self.get_logger().error(f"Model TF failed: {result.err_msg}")
            self.go_to_state(RobotStateId.WAITING_FOR_AUX)
    
    
    def kill_callback(self, msg: Bool):
        self.states[RobotStateId.WAITING_FOR_KILL].passed = not msg.data
    
    
    def aux_callback(self, msg: Bool):
        if not msg.data:
            self.states[RobotStateId.WAITING_FOR_AUX].passed = True
            
            if self.current_state == RobotStateId.WAITING_FOR_TREE:
                self.send_goal_tree()
                
            if self.current_state == RobotStateId.TREE_STARTED:
                self.go_to_state(RobotStateId.WAITING_FOR_AUX, True)
            

def main(args=None):
    rclpy.init(args=args)
    state_machine = StateMachine()
    rclpy.spin(state_machine)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    state_machine.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
