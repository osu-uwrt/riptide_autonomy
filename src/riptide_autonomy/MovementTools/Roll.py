#! /usr/bin/env python

import rospy
import smach
import riptide_controllers.msg
import riptide_autonomy.msg
import actionlib
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

class Roll(EventState):
    """
    Handles rotating the robot's roll.
    Roll is like tilting the side.

    @param angle => float
        the angle to rotate by
        
    """
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['Success', 'Failure'],
            input_keys=['args'])
        self.topic = "go_to_roll"
        self.client = ProxyActionClient({
            self.topic:riptide_controllers.msg.GoToRollAction})
        #self.client.wait_for_server()

    
    def on_enter(self,userdata):
        Logger.loginfo('Rolling with angle %f'%userdata.args['angle'])
        self.client.send_goal(self.topic, riptide_controllers.msg.GoToRollGoal(userdata.args['angle']))

    def execute(self, userdata):
        if self.client.has_result(self.topic):
            result = self.client.get_result(self.topic)
            status = 'Success'       
            return status