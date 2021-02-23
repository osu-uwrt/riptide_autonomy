#! /usr/bin/env python3

#import rospy
import smach
from flexbe_core import EventState, Logger
from MovementTools.Translate import Translate
from MovementTools.Depth import Depth
from MovementTools.Pitch import Pitch
from MovementTools.Roll import Roll
from MovementTools.Yaw import Yaw
from MovementTools.GateManuever import GateManuever

class Move(EventState):
    """
    Handles all possible movements of the robot given the proper movement data.

    @param type => String
        indication of what type of movement to call via this statemachine
    @param args => Dictionary
        maps the required data types for that movement. See substates for details.
    
    """
    def __init__(self,target_time):
        smach.State.__init__(self, 
            outcomes=['Success', 'Failure'],
            input_keys=['type','args'])
    
    def execute(self, userdata):
        status = 'Success'
        sm = smach.StateMachine(outcomes=['Success', 'Failure'])
        sm.userdata.args = userdata.args
        if userdata.type == 'translate':
            with sm:
                smach.StateMachine.add('TRANSLATION', Translate(),
                                        transitions={'Success': 'Success', 'Failure' : 'Failure'},
                                        remapping={'args':'args'})
                status = sm.execute()
        elif userdata.type == 'lqr':
            #TODO Implement LQR
            Logger.loginfo('ERROR: LQR is not implemented yet')
            status = 'Failure'
        elif userdata.type == 'depth':
            with sm:
                smach.StateMachine.add('DEPTH', Depth(),
                                        transitions={'Success': 'Success', 'Failure' : 'Failure'},
                                        remapping={'args':'args'})
                status = sm.execute()
        elif userdata.type == 'yaw':
            with sm:
                smach.StateMachine.add('YAW', Yaw(),
                                        transitions={'Success': 'Success', 'Failure' : 'Failure'},
                                        remapping={'args':'args'})
                status = sm.execute()
        elif userdata.type == 'pitch':
            with sm:
                smach.StateMachine.add('PITCH', Pitch(),
                                        transitions={'Success': 'Success', 'Failure' : 'Failure'},
                                        remapping={'args':'args'})
                status = sm.execute()
        elif userdata.type == 'roll':
            with sm:
                smach.StateMachine.add('ROLL', Roll(),
                                        transitions={'Success': 'Success', 'Failure' : 'Failure'},
                                        remapping={'args':'args'})
                status = sm.execute()
        elif userdata.type == 'gateManuever':
            with sm:
                smach.StateMachine.add('GATEMANUEVER', GateManuever(),
                                        transitions={'Success':'Success', 'Failure':'Failure'})
                status = sm.execute()
        else:
            Logger.loginfo('ERROR: type of %s not recognized'%userdata.type)
        return status
