#! /usr/bin/env python

import rospy
import smach
import riptide_controllers.msg
import riptide_autonomy.msg
import actionlib

class Yaw(smach.State):
    """
    Handles rotating the robot's yaw by a given angle.
    Yaw is like left/right movement.

    @param angle => float
        the angle to rotate by
        
    """
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['Success', 'Failure'],
            input_keys=['args'])
    
    def execute(self, userdata):
        status = 'Success'
        rospy.loginfo('Yawing with angle %f'%userdata.args['angle'])
        client = actionlib.SimpleActionClient(
            "go_to_yaw", riptide_controllers.msg.GoToYawAction)
        client.wait_for_server()
        client.send_goal(riptide_controllers.msg.GoToYawGoal(userdata.args['angle']))
        return status