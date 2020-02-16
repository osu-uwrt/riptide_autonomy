import sys
sys.path.insert(0, '../states/')
from states import Move
import smach
import rospy
import time


if __name__ == '__main__':
    rospy.init_node('translation_test')
    sm = smach.StateMachine(outcomes=['Success', 'Failure'])
    with sm:
        sm.userdata.type = 'translation'
        sm.userdata.args = {'x': 10, 'y':10}
        smach.StateMachine.add('MOVE', Move())
        sm.execute()
        time.sleep(2)
        sm.userdata.args = {'x':-10, 'y':-10}
        sm.execute()