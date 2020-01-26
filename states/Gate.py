#! /usr/bin/env python

import rospy
import smach
from Move import *

def main():
    rospy.init_node('movement_test')
    sm = smach.StateMachine(outcomes=['done', 'fail'])
    sm.userdata.args = {'x' : 2, 'y' : 2, 'angle' : 36.4, 'depth' : 3.14}
    with sm:
        sm.userdata.type = 'move'
        smach.StateMachine.add('MOVE', Move(),
                                transitions={'Success': 'done',
                                            'Failure': 'fail'},
                                remapping={'type':'type','args':'args'})
        sm.execute()
        sm.userdata.type = 'depth'
        sm.execute()
        sm.userdata.type = 'yaw'
        sm.execute()
        sm.userdata.type = 'pitch'
        sm.execute()
        sm.userdata.type = 'roll'
        sm.execute()

if __name__ == '__main__':
    main()
