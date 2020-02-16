import sys
sys.path.insert(0, '../states/')
from states import Move
import smach
import rospy
import time


if __name__ == '__main__':
    rospy.init_node('depth_test')
    sm = smach.StateMachine(outcomes=['Success', 'Failure'])
    with sm:
        sm.userdata.type = 'depth'
        sm.userdata.args = {'depth': 3}
        smach.StateMachine.add('MOVE', Move())
        sm.execute()
        time.sleep(2)
        sm.userdata.args = {'depth':2}
        sm.execute()