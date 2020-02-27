#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from riptide_states.big_depth_state import BigDepthState
from riptide_states.big_translate_state import BigTranslateState
from riptide_states.big_align_state import BigAlignState
from riptide_states.big_distance_state import BigDistanceState
from riptide_states.big_gate_maneuver_state import BigGateManeuverState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Feb 23 2020
@author: BigDawgsUWRT
'''
class gate_taskSM(Behavior):
	'''
	does the gate task, badly
	'''


	def __init__(self):
		super(gate_taskSM, self).__init__()
		self.name = 'gate_task'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:186 y:551, x:842 y:576
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.depth = -0.65
		_state_machine.userdata.x_start = 3
		_state_machine.userdata.y_start = 0
		_state_machine.userdata.bboxWidth = 0.07
		_state_machine.userdata.hold = True
		_state_machine.userdata.obj = 'Gate'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:225 y:136
			OperatableStateMachine.add('gotoDepth',
										BigDepthState(topic="/puddles/go_to_depth"),
										transitions={'Success': 'translate', 'Failure': 'failed'},
										autonomy={'Success': Autonomy.Off, 'Failure': Autonomy.Off},
										remapping={'depth': 'depth'})

			# x:445 y:133
			OperatableStateMachine.add('translate',
										BigTranslateState(topic="puddles/move_distance"),
										transitions={'Success': 'align', 'Failure': 'failed'},
										autonomy={'Success': Autonomy.Off, 'Failure': Autonomy.Off},
										remapping={'x': 'x_start', 'y': 'y_start'})

			# x:664 y:132
			OperatableStateMachine.add('align',
										BigAlignState(topic="/puddles/align"),
										transitions={'Success': 'distance', 'Failure': 'failed'},
										autonomy={'Success': Autonomy.Off, 'Failure': Autonomy.Off},
										remapping={'obj': 'obj', 'bboxWidth': 'bboxWidth', 'hold': 'hold'})

			# x:655 y:278
			OperatableStateMachine.add('distance',
										BigDistanceState(topic="/puddles/get_distance"),
										transitions={'Success': 'translate2', 'Failure': 'failed'},
										autonomy={'Success': Autonomy.Off, 'Failure': Autonomy.Off},
										remapping={'object': 'obj', 'dist': 'dist'})

			# x:441 y:268
			OperatableStateMachine.add('translate2',
										BigTranslateState(topic="/puddles/move_distance"),
										transitions={'Success': 'gateManeuver', 'Failure': 'failed'},
										autonomy={'Success': Autonomy.Off, 'Failure': Autonomy.Off},
										remapping={'x': 'dist', 'y': 'y_start'})

			# x:193 y:268
			OperatableStateMachine.add('gateManeuver',
										BigGateManeuverState(topic="/puddles/gate_maneuver"),
										transitions={'Success': 'finished', 'Failure': 'failed'},
										autonomy={'Success': Autonomy.Off, 'Failure': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
