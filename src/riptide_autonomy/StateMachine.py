#! /usr/bin/env python3

import time
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time 
from nortek_dvl_msgs.msg import DvlStatus
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
#import topic type 
from std_msgs.msg._bool import Bool
from functools import partial
from std_srvs.srv import Empty
from chameleon_tf_msgs.action._model_frame import ModelFrame
from riptide_msgs2.action._execute_tree import ExecuteTree
#immport service type, add dependency maybe

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

#If roll or pitch pass this limit, output an error
rollLimit = 45 #degrees
pitchLimit = 45 #degrees
odometryIsGood = False #Only changes to true once odometry positions aren't going bad, then aux callback runs
class StateMachine(Node):

    #Member variables, keeps their values throughout the class
    linearVelocityX = []
    linearVelocityY = []
    linearVelocityZ = []

    linearAccelerationX = []
    linearAccelerationY = []
    linearAccelerationZ = []

    def __init__(self):
        super().__init__('state_machine')
        self.linearVelocityX = []
        self.linearVelocityY = []
        self.linearVelocityZ = []

        self.linearAccelerationX = 0
        self.linearAccelerationY = 0 
        self.linearAccelerationZ = 0  

        self.subscriptionDVL = self.create_subscription(
            DvlStatus,
            'dvl/status',
            self.dvl_callback,
            10)
        

        self.subscriptionOdometry = self.create_subscription(
            Odometry, #Class
            'odometry/filtered', #This is topic name
            self.odometry_callback,
            10)
        
        self.subscriptionBool = self.create_subscription(
            Bool, #Class
            'state/aux', #This is topic name
            self.aux_callback,
            10)
        
        self._action_clientTag = ActionClient(
            self,
            ModelFrame, #Action type needed
            'model_frame') #Action name needed
        
        self._action_clientRiptide = ActionClient(
            self, 
            ExecuteTree,
            'autonomy/run_tree')
        
        
      
    

    def atLeastThreeTrue(self, arr):
        count = 0
        for value in arr:
            if value:
                count+=1
        if count >= 3:
            return True
        else:
            return False
    
    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4) 
     
        return roll_x,pitch_y,yaw_z# in radians



    
            
    ##FORMAT --> ros2 topic pub <topic_name> <msg_type>
    #typehinting msg:DvlStatus makes sure msg is of type DvlStats
    def dvl_callback(self, msg:DvlStatus):
        statusMessages = [msg.b1_vel_valid, msg.b2_vel_valid, msg.b3_vel_valid, msg.b4_vel_valid]
        listOfStatusMessages = [None]*40
        for i in range(len(listOfStatusMessages)):
            if self.atLeastThreeTrue(statusMessages):
                listOfStatusMessages[i] = True 
            else:
                listOfStatusMessages[i] = False
                
        if all(listOfStatusMessages):
            self.get_logger().info("GOOD STATUS MESSAGES")
            self.odometry_callback()
        else:
            self.get_logger().info("BAD STATUS MESSAGES")

       
        
    def odometry_callback(self,msg:Odometry):
        
        timeInitial = self.get_clock().now()
        linearVelocityX = msg.twist.twist.linear.x
        linearVelocityY = msg.twist.twist.linear.y
        linearVelocityZ = msg.twist.twist.linear.z
        time.sleep(0.001)
        timeFinal = self.get_clock().now()

        timeDifference = timeFinal-timeInitial
        
        
           
        acceleration = math.sqrt()
        if(acceleration<1.5):
            self.aux_callback()
        
        


        roll,pitch,yaw = self.euler_from_quaternion(msg.pose.pose.orientation.x
                                           ,msg.pose.pose.orientation.y
                                           ,msg.pose.pose.orientation.z
                                           ,msg.pose.pose.orientation.w)
        
       



        #Set odometryIsGood to true once all these values are checked

        #FIND A WAY TO SEE IF XYZ IS GOING TO REALLY BIG NUMBERS if()
        #With quaternion, convert to euler, then only worry about pitch and roll, x axis spin and y axis spin
        #TRY DERIVATIVE WAY, STORE THE X Y Z AND STUFF
        #IF THINGS LIKE ACCERLATION, JERK, SNAP, CRACKLE AREN'T CRAZY VALUES
        #THESE VALUES WILL HAVE TO BE CALCULATED, ROBOT ONLY GIVES UP TO VELOCITY
        #SO JUST USE V/T, A/T ETC. 
        #THEN DVL ISNT GOING CRAZY
        #ONLY THEN DO I CALL aux_callback to start tag_cal


    def send_goal_riptide(self,order):
        goal_msg = ExecuteTree.Goal()
        #Set this to an absolute path whatever
        goal_msg.tree = "/riptide_autonomy/trees/CompTree.xml"
        #self._action_clientRiptide.wait_for_server()
        self._send_goal_future = self._action_clientRiptide.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback_riptide)
    
    def goal_response_callback_riptide(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback_riptide)

    def get_result_callback_riptide(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        #rclpy.shutdown()



    def send_goal_tag(self,order):
        goal_msg = ModelFrame.Goal()
        goal_msg.monitor_child = "estimated_origin_frame"
        goal_msg.monitor_parent = "world"
        goal_msg.samples = 10
        #check response message, use callback
        #monitor_child is always estimated_origin_frame
        #montior_parent is always world
        #samples is always 10
        #self._action_clientTag.wait_for_server()

        self._send_goal_future = self._action_clientTag.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback_tag)

    
    def goal_response_callback_tag(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback_tag)

    def get_result_callback_tag(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        #rclpy.shutdown()
    
    #Write a callback to check once send_goal completes,
    #Then write another action client --> behavior tree
    

    def aux_callback(self, msg:Bool):
        if(odometryIsGood):
            auxTrigger = msg.data
            if(auxTrigger):
                self.send_goal_tag()
            

        
def main(args=None):
    rclpy.init(args=args)
    
    state_machine = StateMachine()

    futureTag = state_machine.send_goal_tag(10)
    futureRiptide = state_machine.send_goal_riptide(10)

    rclpy.spin(state_machine)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    state_machine.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

    #NEXT STEP
    #MAKE A SUBSCRIBER AND SUBSCRIBE TO ODOMETRY/FILTERED - A TOPIC, published by localization node
    #CHECK IF ODOMETRY/FILTERED IS PINGING OFF TO INFINITY
    #LOOPS FOREVER
    #ODOMETRY-FILTERED WILL GO OFF INTO INFINITY BASED ON DVL
    #DVL... some other stuff --> BLACK BOX --> ODOMETRY-FILTERED
    #ODOMETRY/FILTERED PUBLISHES X,Y,Z position and Quaternion stuff --> convert to roll, pitch, yaw - do computations on this
    
    #NEXT PART RECOGNIZE AUX SWITCH
    #WHEN NAV IS STARTED (DVL IS IN WATER)
    #IF AUX SWITCH IS TRIGGERED, MAKE REQUEST TO ACTION SERVER
    #AUX WILL BE TRIGGERED (GET THAT INFO FROM A TOPIC)
    #Only difference between action and server client is action gets feedback (like %of aciton that's completed)
    #Create another subscription node to subscribe to the AUX topic, and if the aux trigger is true
    #Call action server to start the tag cal