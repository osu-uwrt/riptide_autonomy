import sys
sys.path.insert(0, '../states/')
from Search import Search
import smach
import rospy
import time

if __name__ == '__main__':
    rospy.init_node('search_test')
    sm = smach.StateMachine(outcomes=['Success', 'Failure'])
    with sm:
        sm.userdata.search_object = 'Gate'
        smach.StateMachine.add('SEARCH', Search())
        sm.execute()