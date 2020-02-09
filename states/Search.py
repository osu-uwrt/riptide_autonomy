#! /usr/bin/env python

import rospy
import smach
from Move import Move
from SearchTools.Distance import Distance
from SearchTools.Align import Align

class Search(smach.State):
    """
    Tries to find a given object and aligns the robot to face it.

    @param search_object => String
        indictation of what to look for.
    """
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['Success', 'Failure'],
            input_keys=['search_object'])
    
    def execute(self, userdata):
        status = 'Success'
        sm = smach.StateMachine(outcomes=['Success', 'Failure'])
        if userdata.search_object == 'Gate':
            with sm:
                sm.userdata.type = 'depth'
                sm.userdata.args = {'depth': .65, 
                                    'x': 2, 'y': -.8, 
                                    'obj':'Gate', 
                                    'bboxWidth':.07, 
                                    'hold':True}
                smach.StateMachine.add('MOVE_DOWN', Move(),
                                transitions={'Success': 'Success',
                                            'Failure': 'Failure'},
                                remapping={'type':'type','args':'args'})
                smach.StateMachine.add('ALIGN', Align(), 
                                            transitions={'Success': 'Success',
                                            'Failure': 'Failure'})
                smach.StateMachine.add('DISTANCE', Distance(),
                                transitions={'Success': 'Success',
                                            'Failure': 'Failure'},
                                remapping={ 'type': 'type',
                                            'args': 'args'})
                smach.StateMachine.add('MOVE_AWAY', Move(),
                                transitions={'Success': 'Success',
                                            'Failure': 'Failure'},
                                remapping={'type':'type',
                                        'args':'args'})
            status = sm.execute()
        else:
            rospy.loginfo('object does not exist')
            status = 'Failure'
        return status
