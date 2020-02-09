#! /usr/bin/env python

import rospy
import smach
import riptide_controllers.msg
import riptide_autonomy.msg
import actionlib

class Align(smach.State):
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
    
    def execute(self, userdata):
        status = 'Success'
        rospy.loginfo('Aligning robot')
        client = actionlib.SimpleActionClient(
            "align", riptide_controllers.msg.AlignAction)
        client.wait_for_server()
        client.send_goal(riptide_controllers.msg.AlignGoal(userdata.args['obj'], userdata.args['bboxWidth'], userdata.args['hold']))
        return status