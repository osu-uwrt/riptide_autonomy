#! /usr/bin/env python

import rospy
import smach
import riptide_controllers.msg
import riptide_autonomy.msg
import actionlib
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

class Yaw(EventState):
    """
    Handles rotating the robot's yaw by a given angle.
    Yaw is like left/right movement.

    @param angle => float
        the angle to rotate by
        
    """
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['Success', 'Failure'],
            input_keys=['args'])
        self.topic = "go_to_yaw"
        self.client = ProxyActionClient({
            self.topic, riptide_controllers.msg.GoToYawAction})
        

    def on_enter(self, userdata):
        Logger.loginfo('Yawing with angle %f'%userdata.args['angle'])
        self.client.send_goal(self.topic, riptide_controllers.msg.GoToYawGoal(userdata.args['angle']))
    
    def execute(self, userdata):
        if self.client.has_result(self.topic):
            result = self.client.get_result(self.topic)
            status = 'Success'       
            return status