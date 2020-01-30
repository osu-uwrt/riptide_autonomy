#! /usr/bin/env python

import rospy
import smach
import riptide_controllers.msg
import riptide_autonomy.msg
import actionlib

class Roll(smach.State):
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
    
    def execute(self, userdata):
        status = 'Success'
        rospy.loginfo('Rolling with angle %f'%userdata.args['angle'])
        client = actionlib.SimpleActionClient(
            "go_to_roll", riptide_controllers.msg.GoToRollAction)
        client.wait_for_server()
        client.send_goal(riptide_controllers.msg.GoToRollGoal(userdata.args['angle']))
        return status