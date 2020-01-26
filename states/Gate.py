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
        sm.userdata.type = 'depth'
        smach.StateMachine.add('MOVE', Move(),
                                transitions={'Success': 'done',
                                            'Failure': 'fail'},
                                remapping={'type':'type','args':'args'})
        sm.userdata.type = 'yaw'
        smach.StateMachine.add('MOVE', Move(),
                                transitions={'Success': 'done',
                                            'Failure': 'fail'},
                                remapping={'type':'type','args':'args'})
        sm.userdata.type = 'pitch'
        smach.StateMachine.add('MOVE', Move(),
                                transitions={'Success': 'done',
                                            'Failure': 'fail'},
                                remapping={'type':'type','args':'args'})
        sm.userdata.type = 'roll'
        smach.StateMachine.add('MOVE', Move(),
                                transitions={'Success': 'done',
                                            'Failure': 'fail'},
                                remapping={'type':'type','args':'args'})
    outcome = sm.execute()

if __name__ == '__main__':
    main()
