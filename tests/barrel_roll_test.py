import sys
sys.path.insert(0, '../states/')
from Move import Move
import smach
import rospy
import time


if __name__ == '__main__':
    rospy.init_node('barrel_roll_test')
    sm = smach.StateMachine(outcomes=['Success', 'Failure'])
    with sm:
        sm.userdata.type = 'gateManuever'
        sm.userdata.args = {}
        smach.StateMachine.add('MOVE', Move())
        sm.execute()