#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from riptide_autonomy.bigsearch_sm import BigSearchSM
from riptide_states.big_move_state import BigMoveState
from riptide_states.find_task_location import FindTaskLocation
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Feb 01 2021
@author: Toby McGuire
'''
class GateTaskSM(Behavior):
	'''
	does the gate task better
	'''


	def __init__(self):
		super(GateTaskSM, self).__init__()
		self.name = 'GateTask'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(BigSearchSM, 'BigSearch')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1190 y:140, x:440 y:336
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.x = 0
		_state_machine.userdata.y = 0
		_state_machine.userdata.z = 0
		_state_machine.userdata.orientation = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:150 y:90
			OperatableStateMachine.add('FindGate',
										FindTaskLocation(target="/puddles/mapping.gate"),
										transitions={'Success': 'Move', 'Failed': 'failed'},
										autonomy={'Success': Autonomy.Off, 'Failed': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'z': 'z', 'orientation': 'orientation'})

			# x:685 y:70
			OperatableStateMachine.add('FindExactLocation',
										FindTaskLocation(target="/puddles/mapping.gate"),
										transitions={'Success': 'MoveToGate', 'Failed': 'failed'},
										autonomy={'Success': Autonomy.Off, 'Failed': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'z': 'z', 'orientation': 'orientation'})

			# x:327 y:86
			OperatableStateMachine.add('Move',
										BigMoveState(x=x, y=y, z=z, orientation=None),
										transitions={'done': 'BigSearch', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:902 y:126
			OperatableStateMachine.add('MoveToGate',
										BigMoveState(x=x, y=y, z=z, orientation=orientation),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:497 y:75
			OperatableStateMachine.add('BigSearch',
										self.use_behavior(BigSearchSM, 'BigSearch'),
										transitions={'finished': 'FindExactLocation', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
