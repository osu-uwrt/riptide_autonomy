#! /usr/bin/env python

import rospy
import smach
from Move import Move
from Search import Search

def main():
    rospy.init_node('gate_task')
    sm = smach.StateMachine(outcomes=['done', 'fail'])
    with sm:
        sm.userdata.search_object = 'Gate'
        smach.StateMachine.add('SEARCH', Search(),
                                transitions={'Success': 'done', 'Failure': 'fail'})
        sm.userdata.type = 'gateManuever'
        sm.userdata.args = {}
        smach.StateMachine.add('MOVE', Move())
        sm.execute()

if __name__ == '__main__':
    main()
