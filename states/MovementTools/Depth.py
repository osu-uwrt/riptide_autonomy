#! /usr/bin/env python

import rospy
import smach
import riptide_controllers.msg
import riptide_autonomy.msg
import actionlib

class Depth(smach.State):
    """
    Handles moving the robot to a given depth in meters.

    @param depth => float
        the depth in meters the robot is aiming for
        
    """
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['Success', 'Failure'],
            input_keys=['args'])
    
    def execute(self, userdata):
        status = 'Success'
        rospy.loginfo('Moving to depth %f'%userdata.args['depth'])
        client = actionlib.SimpleActionClient(
            "go_to_depth", riptide_controllers.msg.GoToDepthAction)
        client.wait_for_server()
        client.send_goal(riptide_controllers.msg.GoToDepthGoal(userdata.args['depth']))
        return status