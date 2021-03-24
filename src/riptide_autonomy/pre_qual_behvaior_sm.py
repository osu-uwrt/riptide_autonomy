#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from riptide_states.GetFrontOf import GetFrontOf
from riptide_states.big_move_state import BigMoveState
from riptide_states.flatten_state import FlattenState
from riptide_states.position_parameter_state import PositionParameterState
from riptide_states.relative_move_state import RelativeMoveState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Mar 20 2021
@author: Toby McGuire
'''
class PreQualBehvaiorSM(Behavior):
	'''
	Does the pre qualification movement
	'''


	def __init__(self):
		super(PreQualBehvaiorSM, self).__init__()
		self.name = 'Pre-Qual Behvaior'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:223 y:246
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.x = 0
		_state_machine.userdata.y = 0
		_state_machine.userdata.z = 0
		_state_machine.userdata.orientation = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('Flatten',
										FlattenState(),
										transitions={'Success': 'Submerge', 'Failure': 'failed'},
										autonomy={'Success': Autonomy.Off, 'Failure': Autonomy.Off})

			# x:594 y:466
			OperatableStateMachine.add('GetOrigin',
										PositionParameterState(x=0, y=0, z=-1),
										transitions={'Success': 'GoBackToGate', 'Failure': 'failed'},
										autonomy={'Success': Autonomy.Off, 'Failure': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'z': 'z', 'orientation': 'orientation'})

			# x:363 y:489
			OperatableStateMachine.add('GoBackToGate',
										BigMoveState(),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'z': 'z', 'orientation': 'orientation'})

			# x:827 y:138
			OperatableStateMachine.add('MovePoleBehind',
										RelativeMoveState(x=3, y=-3, z=0),
										transitions={'Success': 'MovePoleRight', 'Failure': 'failed'},
										autonomy={'Success': Autonomy.Off, 'Failure': Autonomy.Off})

			# x:720 y:48
			OperatableStateMachine.add('MovePoleLeft',
										RelativeMoveState(x=3, y=3, z=0),
										transitions={'Success': 'MovePoleBehind', 'Failure': 'failed'},
										autonomy={'Success': Autonomy.Off, 'Failure': Autonomy.Off})

			# x:774 y:303
			OperatableStateMachine.add('MovePoleRight',
										RelativeMoveState(x=-3, y=3, z=0),
										transitions={'Success': 'GetOrigin', 'Failure': 'failed'},
										autonomy={'Success': Autonomy.Off, 'Failure': Autonomy.Off})

			# x:583 y:41
			OperatableStateMachine.add('MoveToPole',
										BigMoveState(),
										transitions={'done': 'MovePoleLeft', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'z': 'z', 'orientation': 'orientation'})

			# x:177 y:37
			OperatableStateMachine.add('Submerge',
										RelativeMoveState(x=0, y=0, z=-1),
										transitions={'Success': 'Find Pole', 'Failure': 'failed'},
										autonomy={'Success': Autonomy.Off, 'Failure': Autonomy.Off})

			# x:384 y:41
			OperatableStateMachine.add('Find Pole',
										GetFrontOf(target="pole_frame"),
										transitions={'Success': 'MoveToPole'},
										autonomy={'Success': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'z': 'z', 'orientation': 'orientation'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
