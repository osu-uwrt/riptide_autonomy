import rospy
import smach
from Move import *

def main():
    rospy.init_node('movement_test')
    sm = smach.StateMachine(outcomes=['done', 'fail'])
    sm.userdata.args = {'x' : 1, 'y' : 2}
    sm.type = 'move'

    with sm:
        smach.StateMachine.add('MOVE', Move(),
                                transitions={'Success': 'done',
                                            'Failure': 'fail'},
                                remapping={'type':'type','args':'args'})
    outcome = sm.execute()

if __name__ == '__main__':
    main()
