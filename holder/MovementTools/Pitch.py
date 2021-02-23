#! /usr/bin/env python3

import rospy
import smach
import riptide_controllers.msg
import riptide_autonomy.msg
import actionlib
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

class Pitch(EventState):
    """
    Handles rotating the robot's pitch.
    Pitch is like tilting up/down.

    @param angle => float
        the angle to rotate by
        
    """
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['Success', 'Failure'],
            input_keys=['args'])
        self.topic = "go_to_pitch"
        self.client = ProxyActionClient({
            self.topic:riptide_controllers.msg.GoToPitchAction})
        #self.client.wait_for_server()

    def on_enter(self, userdata):
        self.client.send_goal(self.topic, riptide_controllers.msg.GoToPitchGoal(userdata.args['angle']))
        Logger.loginfo('Pitching with angle %f'%userdata.args['angle'])

    def execute(self, userdata):
       if self.client.has_result(self.topic):
            result = self.client.get_result(self.topic)
            status = 'Success'       
            return status