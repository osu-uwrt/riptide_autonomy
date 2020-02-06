#! /usr/bin/env python

import rospy
import smach
import riptide_controllers.msg
import riptide_autonomy.msg
import actionlib

class Distance(smach.State):
    """
    Finds the distance between the robot and a given object.

    @param obj => string
        the name of the object
        
    """
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['Success', 'Failure'],
            input_keys=['obj'],
            output_keys=['type', 'args'])
    
    def execute(self, userdata):
        status = 'Success'
        client = actionlib.SimpleActionClient(
        "get_distance", riptide_controllers.msg.GetDistanceAction)
        client.wait_for_server()
        client.send_goal(riptide_controllers.msg.GetDistanceGoal(userdata.obj))
        client.wait_for_result()
        dist = client.get_result().distance
        userdata.type = 'translate'
        userdata.args = {'x': dist - 2, 'y': 0}
        return status