#! /usr/bin/env python

import rospy
import smach
import riptide_controllers.msg
import riptide_autonomy.msg
import actionlib
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

class GateManuever(EventState):
    """
    Fany way of getting through the gate
    """
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['Success', 'Failure'])
        self.topic = "gate_maneuver"
        self.client = ProxyActionClient({
            self.topic:riptide_controllers.msg.GateManeuverAction})
        #self.client.wait_for_server()
        
    def on_enter(self, userdata):
        self.client.send_goal(self.topic, riptide_controllers.msg.GateManeuverGoal())

    def execute(self, userdata):
        if self.client.has_result(self.topic):
            result = self.client.get_result(self.topic)
            status = 'Success'       
            return status