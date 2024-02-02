#! /usr/bin/env python3
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time 
from nortek_dvl_msgs.msg import DvlStatus
from nav_msgs.msg import Odometry
#import topic type 
from std_msgs.msg._bool import Bool
from functools import partial
from std_srvs.srv import Empty
from chameleon_tf_msgs.action._model_frame import ModelFrame
from riptide_msgs2.action._execute_tree import ExecuteTree
#immport service type, add dependency maybe

#start nav
#first get good DVL status message, 
#then listen to all following DVL messages for ~5 seconds
#this ensures nav doesn't start due to wrong DVL status

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
        
        self._action_clientTag = StateMachine(
            self,
            ModelFrame, #Action type needed
            'model_frame') #Action name needed
        
        self._action_clientRiptide = StateMachine(
            self, 
            ExecuteTree,
            'riptide_msgs2.action._execute_tree')
        
        
        #riptide_messages2.action.executetree
        #autonomy/run_tree
        self.subscription  # prevent unuseinearJerkX and linearJerkY and linearJerkZ < d variable warning
    

    def atLeastThreeTrue(self, arr):
        count = 0
        for i in range(len(arr)):
            if i == True:
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



    #parameters need to be parameteres inside the imported service type
    def call_set_pen_service(self):
        #add service type in client()
        # client(service type, service name)
        client = self.create_client(Empty, "/enable")

        #wait for the service, checks if server is up and running
        while not client.wait_for_service(1.0):
            #warn() prints in yellow collor
            self.get_logger().warn("Waiting for service...")
        
        #trigger to tell go
        request = Empty.Request()
        
        #async makes the call return immediately 
        #future is something thats done in the future
        #can do stuff with future like add callback so when u get response from service then future calls callback
        future = client.call_async(request)

        #code below calls the callback_set_pen when response is given
        future.add_done_callback(partial(self.callback_set_pen))


    def callback_set_pen(self, future):
        #callback for when service replies 
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,))                             
       
    #ros2 topic pub -r8 /dvl/status nortek_dvl_msgs/msg/DvlStatus
    #ros2 topic pub -r8 /odometry/filtered nav_msgs/msg/Odometry
    #ros2 topic pub -r8 /state/aux std_msgs.msg/_bool/Bool
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
        else:
            self.get_logger().info("BAD STATUS MESSAGES")

        #add the new function for client service here to do stuff
        if not all(listOfStatusMessages):
            self.get_logger().info("NOT GOOD TO GO")
            self.call_set_pen_service()
        
    def odometry_callback(self,msg:Odometry):
        

        timeInitial = self.get_clock().now()
        time.sleep(0.001)
        timeFinal = self.get_clock().now()

        timeDifference = timeFinal-timeInitial
        
        #100% can partition this into diff methods 
        linearVelocityX = msg.twist.twist.linear.x
        linearVelocityY = msg.twist.twist.linear.y
        linearVelocityZ = msg.twist.twist.linear.z
        self.linearVelocityX.append(linearVelocityX)
        self.linearVelocityY.append(linearVelocityY)
        self.linearVelocityZ.append(linearVelocityZ)

        if(len(self.linearVelocityX)>=2):
            linearAccelerationX = (self.linearVelocityX.index(len(self.linearVelocityX)-1)-self.linearVelocityX.index(len(self.linearVelocityX)-2))/timeDifference
            linearAccelerationY = (self.linearVelocityY.index(len(self.linearVelocityY)-1)-self.linearVelocityY.index(len(self.linearVelocityY)-2))/timeDifference
            linearAccelerationZ = (self.linearVelocityZ.index(len(self.linearVelocityZ)-1)-self.linearVelocityZ.index(len(self.linearVelocityZ)-2))/timeDifference
        
        acceleration = math.sqrt(linearAccelerationX**2+linearAccelerationY**2+linearAccelerationZ**2)


        roll,pitch,yaw = self.euler_from_quaternion(msg.pose.pose.orientation.x
                                           ,msg.pose.pose.orientation.y
                                           ,msg.pose.pose.orientation.z
                                           ,msg.pose.pose.orientation.w)
        
        if(acceleration<1.5):
            self.aux_callback()



        #Set odometryIsGood to true once all these values are checked

        #FIND A WAY TO SEE IF XYZ IS GOING TO REALLY BIG NUMBERS if()
        #With quaternion, convert to euler, then only worry about pitch and roll, x axis spin and y axis spin
        #TRY DERIVATIVE WAY, STORE THE X Y Z AND STUFF
        #IF THINGS LIKE ACCERLATION, JERK, SNAP, CRACKLE AREN'T CRAZY VALUES
        #THESE VALUES WILL HAVE TO BE CALCULATED, ROBOT ONLY GIVES UP TO VELOCITY
        #SO JUST USE V/T, A/T ETC. 
        #THEN DVL ISNT GOING CRAZY
        #ONLY THEN DO I CALL aux_callback to start tag_cal


    def send_goal_riptide(self):
        goal_msg = ExecuteTree.Goal()
        #Set this to an absolute path whatever
        goal_msg.tree = "/riptide_autonomy/trees/CompTree.xml"
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
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



    def send_goal_tag(self):
        goal_msg = ModelFrame.Goal()
        goal_msg.monitor_child = "estimated_origin_frame"
        goal_msg.monitor_parent = "world"
        goal_msg.samples = 10
        #check response message, use callback
        #monitor_child is always estimated_origin_frame
        #montior_parent is always world
        #samples is always 10
        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
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