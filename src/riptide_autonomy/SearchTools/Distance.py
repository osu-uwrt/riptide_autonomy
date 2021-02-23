#! /usr/bin/env python3

import rospy
import smach
import riptide_controllers.msg
import riptide_autonomy.msg
import actionlib
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

class Distance(EventState):
    """
    Finds the distance between the robot and a given object.

    @param obj => string
        the name of the object
        
    """
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['Success', 'Failure'],
            input_keys=['args'],
            output_keys=['type', 'args'])
        self.topic = "get_distance"
        self.client = ProxyActionClient({
            self.topic:riptide_controllers.msg.GetDistanceAction})
        
    def on_enter(self, userdata):
        Logger.loginfo('Moving %f m away from %s'%(2, userdata.args['obj']))
        self.client.send_goal(self.topic, riptide_controllers.msg.GetDistanceGoal(userdata.args['obj']))
        
    def execute(self, userdata):        
        if self.client.has_result(self.topic):
            dist = client.get_result().distance
            status = 'Success'
            return status
        
        
        