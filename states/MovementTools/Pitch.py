#! /usr/bin/env python

import rospy
import smach
import riptide_controllers.msg
import riptide_autonomy.msg
import actionlib

class Pitch(smach.State):
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
    
    def execute(self, userdata):
        status = 'Success'
        rospy.loginfo('Pitching with angle %f'%userdata.args['angle'])
        client = actionlib.SimpleActionClient(
            "go_to_pitch", riptide_controllers.msg.GoToPitchAction)
        client.wait_for_server()
        client.send_goal(riptide_controllers.msg.GoToPitchGoal(userdata.args['angle']))
        return status