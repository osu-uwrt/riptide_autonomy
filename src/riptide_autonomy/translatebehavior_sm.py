#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from riptide_states.big_translate_parameter_state import BigTranslateParameterState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Mar 01 2020
@author: UWRT
'''
class TranslateBehaviorSM(Behavior):
	'''
	Translate using parameters
	'''


	def __init__(self):
		super(TranslateBehaviorSM, self).__init__()
		self.name = 'TranslateBehavior'

		# parameters of this behavior
		self.add_parameter('x', 0)
		self.add_parameter('y', 0)
		self.add_parameter('topic', '/puddles/move_distance')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:62 y:123
			OperatableStateMachine.add('Translate',
										BigTranslateParameterState(topic=self.topic, x=self.x, y=self.y),
										transitions={'Success': 'finished', 'Failure': 'failed'},
										autonomy={'Success': Autonomy.Off, 'Failure': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
