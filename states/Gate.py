#! /usr/bin/env python

import rospy
import smach
from Move import Move
from Search import Search

def main():
    rospy.init_node('movement_test')
    sm = smach.StateMachine(outcomes=['done', 'fail'])
    with sm:
        smach.StateMachine.add('SEARCH', Search(),
                                transitions={'Success': 'done', 'Failure': 'fail'})
        


if __name__ == '__main__':
    main()
