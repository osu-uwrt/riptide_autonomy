#! /usr/bin/env python3

import rospy
import smach
import riptide_controllers.msg
import riptide_autonomy.msg
import actionlib
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

class Translate(EventState):
    """
    Handles simple translations of the robot across the xy plane of the robot
    oriented with the reference frame of it being upright.

    @param x => float
        the distance to move in the x direction
    @param y => float
        the distance to move in the y direction

    """
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['Success', 'Failure'],
            input_keys=['args'])
        self.topic = "move_distance"
        self.client = ProxyActionClient({
            self.topic:riptide_controllers.msg.MoveDistanceAction})
        

    def on_enter(self, userdata):
        Logger.loginfo('Translating by vector <%f, %f>'%(userdata.args['x'], userdata.args['y']))
        self.client.send_goal(self.topic, riptide_controllers.msg.MoveDistanceGoal(userdata.args['x'], userdata.args['y']))
    def execute(self, userdata):
         if self.client.has_result(self.topic):
            result = self.client.get_result(self.topic)
            status = 'Success'       
            return status