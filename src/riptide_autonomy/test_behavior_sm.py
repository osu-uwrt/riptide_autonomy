#!/usr/bin/env python3
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.input_state import InputState
from flexbe_states.decision_state import DecisionState
from flexbe_states.calculation_state import CalculationState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Feb 05 2020
@author: cedric
'''
class test_behaviorSM(Behavior):
	'''
	testtttttttttttttttttttt
	'''


	def __init__(self):
		super(test_behaviorSM, self).__init__()
		self.name = 'test_behavior'

		# parameters of this behavior
		self.add_parameter('yes', False)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:599 y:427
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:378 y:183
			OperatableStateMachine.add('start',
										InputState(request=integer, message=eeeee),
										transitions={'received': 'eeeeeeeeeee', 'aborted': 'failed', 'no_connection': 'eeeeeeeeeee', 'data_error': 'finished'},
										autonomy={'received': Autonomy.Off, 'aborted': Autonomy.Off, 'no_connection': Autonomy.Off, 'data_error': Autonomy.Off},
										remapping={'data': 'data'})

			# x:1115 y:242
			OperatableStateMachine.add('eeeeeeeeeee',
										DecisionState(outcomes="LOL", conditions=input_value > 0),
										transitions={'LOL': 'failed'},
										autonomy={'LOL': Autonomy.Off},
										remapping={'input_value': 'data'})

			# x:808 y:383
			OperatableStateMachine.add('lmFAOR',
										CalculationState(calculation=54x-5w),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'input_value', 'output_value': 'output_value'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
