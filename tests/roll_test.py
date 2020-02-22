import sys
sys.path.insert(0, '../states/')
from Move import Move
import smach
import rospy
import time


if __name__ == '__main__':
    rospy.init_node('roll_test')
    sm = smach.StateMachine(outcomes=['Success', 'Failure'])
    with sm:
        sm.userdata.type = 'roll'
        sm.userdata.args = {'angle': 45}
        smach.StateMachine.add('MOVE', Move())
        sm.execute()