#! /usr/bin/env python

import rospy
import smach
from actionTools import *

class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['Success', 'Failure'],
            input_keys=['type','args'])
    
    def execute(self, userdata):
        status = 'Failure'
        if userdata.type == 'move':
            rospy.loginfo('Moving by vector <%f, %f>'%(userdata.args['x'], userdata.args['y']))
            moveAction(userdata.args['x'], userdata.args['y'])
            status = 'Success'
        elif userdata.type == 'depth':
            rospy.loginfo('Moving to depth %f'%userdata.args['depth'])
            depthAction(userdata.args['depth'])
            status = 'Success'
        elif userdata.type == 'yaw':
            rospy.loginfo('Yawing with angle %f'%userdata.args['angle'])
            yawAction(userdata.args['angle'])
            status = 'Success'
        elif userdata.type == 'pitch':
            rospy.loginfo('Pitching with angle %f'%userdata.args['angle'])
            pitchAction(userdata.args['angle'])
            status = 'Success'
        elif userdata.type == 'roll':
            rospy.loginfo('Rolling with angle %f'%userdata.args['angle'])
            rollAction(userdata.args['angle'])
            status = 'Success'
        else:
            rospy.loginfo('ERROR: type of %s not recognized'%userdata.type)
        return status