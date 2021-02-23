#! /usr/bin/env python3

import rospy
import smach
import riptide_controllers.msg
import riptide_autonomy.msg
import actionlib
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

class Align(EventState):
    """
    Aligns robot to an object.

    @param obj => string
        the name of the object

    @param bboxWidth => float
        ratio of how big the bbox is to the camera input

    @param hold => boolean
        determines whether or not to stay in position
        
    """
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['Success', 'Failure'],
            input_keys=['args'])
        self.topic = "align"
        self.client = ProxyActionClient({
            self.topic:riptide_controllers.msg.AlignAction})

    def on_enter(self, userdata):
        Logger.loginfo('Aligning robot')
        self.client.send_goal(self.topic, riptide_controllers.msg.AlignGoal(userdata.args['obj'], userdata.args['bboxWidth'], userdata.args['hold']))

    def execute(self, userdata):
        if self.client.has_result(self.topic):
            result = self.client.get_result(self.topic)
            status = 'Success'       
            return status