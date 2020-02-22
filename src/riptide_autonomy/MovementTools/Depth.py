#! /usr/bin/env python

import rospy
import smach
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
import riptide_controllers.msg
import riptide_autonomy.msg
import actionlib

class Depth(EventState):
    """
    Handles moving the robot to a given depth in meters.

    @param depth => float
        the depth in meters the robot is aiming for
        
    """
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['Success', 'Failure'],
            input_keys=['args'])
        self.client = ProxyActionClient({"go_to_depth": riptide_controllers.msg.GoToDepthAction})
        
    def on_enter(self, userdata):
        Logger.loginfo('Moving to depth %f'%userdata.args['depth'])
        
        #self.client.wait_for_server()
        self.client.send_goal("go_to_depth", riptide_controllers.msg.GoToDepthGoal(userdata.args['depth']))

    def execute(self, userdata):
        if self.client.has_result("go_to_depth"):
            result = self.client.get_result("go_to_depth")
            status = 'Success'       
            return status