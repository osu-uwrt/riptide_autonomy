#! /usr/bin/env python

import rospy
import smach
import riptide_controllers.msg
import riptide_autonomy.msg
import actionlib

class GateManuever(smach.State):
    """
    Fany way of getting through the gate
    """
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['Success', 'Failure'])
    
    def execute(self, userdata):
        status = 'Success'
        rospy.loginfo('Look at this! Gate maneuver!!')
        client = actionlib.SimpleActionClient(
            "gate_maneuver", riptide_controllers.msg.GateManeuverAction)
        client.wait_for_server()
        client.send_goal(riptide_controllers.msg.GateManeuverGoal())
        return status