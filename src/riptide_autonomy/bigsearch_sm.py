#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                   #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from riptide_states.big_gate_maneuver_state import BigGateManeuverState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import rospy 
import smach ## state machine
from Move import Move 
from SearchTools.Distance import Distance
from SearchTools.Align import Align 
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
# [/MANUAL_IMPORT]


'''
Created on Mon Jan 18 2021
@author: Toby McGuire

Edited Fri Jan 22 2021
@author: Evie Marx
'''
class BigSearchSM(Behavior):
	'''
	Search
	'''


	def __init__(self):
		super(BigSearchSM, self).__init__()
		self.name = 'BigSearch'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
        smach.State.__init__(self,
        outcomes=['finished', 'failed'], io_keys{'searchForObject'})
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
        if userdata.searchForObject == 'Gate':
            with sm:
                sm.sm.userdata.type = 'depth'
                sm.userdata.args = {'depth': .65, 
                                    'x': 2, 'y': -.8, 
                                    'obj':'Gate', 
                                    'bboxWidth':.07, 
                                    'hold':True}  # lines 63-67 use the numbers for depths, x, and y 
                                    # used in the master branch of Search.py - I am not sure what numbers we should use here
                smach.StateMachine.add('MOVE_DOWN', Move(),
                                transitions={'finished': 'finished',
                                            'failed': 'failed'},
                                remapping={'type':'type','args':'args'})
                smach.StateMachine.add('ALIGN', Align(), 
                                            transitions={'finished': 'finished',
                                            'failed': 'failed'},
                smach.StateMachine.add('DISTANCE', Distance(),
                                transitions={'finished': 'finished',
                                            'failed': 'failed'},
                                remapping={ 'type': 'type',
                                            'args': 'args'})
                smach.StateMachine.add('MOVE_AWAY', Move(),
                                transitions={'finished': 'finished',
                                            'failed': 'failed'},
                                remapping={'type':'type',
                                        'args':'args'})
            status = sm.execute()
         else:
            Logger.loginfo('object does not exist')
            status = 'failed'
        return status   
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:60 y:136
			OperatableStateMachine.add('tmp',
										BigGateManeuverState(topic=bbox),
										transitions={'Success': 'finished', 'Failure': 'failed'},
										autonomy={'Success': Autonomy.Off, 'Failure': Autonomy.Off})


		return _state_machine
	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]

	# [/MANUAL_FUNC]