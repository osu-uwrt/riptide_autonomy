#! /usr/bin/env python

import rospy
import smach
import riptide_controllers.msg
import riptide_autonomy.msg
import actionlib

class Translate(smach.State):
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
    
    def execute(self, userdata):
        status = 'Success'
        rospy.loginfo('Translating by vector <%f, %f>'%(userdata.args['x'], userdata.args['y']))
        client = actionlib.SimpleActionClient(
            "move_distance", riptide_controllers.msg.MoveDistanceAction)
        client.wait_for_server()
        client.send_goal(riptide_controllers.msg.MoveDistanceGoal(userdata.args['x'], userdata.args['y']))
        return status