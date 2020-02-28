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
from riptide_states.big_pitch_state import BigPitchState
from riptide_states.big_roll_state import BigRollState
from riptide_states.big_yaw_state import BigYawState
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
		_state_machine.userdata.initRollAngle = 0
		_state_machine.userdata.initPitchAngle = 0
		_state_machine.userdata.initYawAngle = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:225 y:136
			OperatableStateMachine.add('gotoDepth',
										BigDepthState(topic="/puddles/go_to_depth"),
										transitions={'Success': 'setPitch0State', 'Failure': 'failed'},
										autonomy={'Success': Autonomy.Off, 'Failure': Autonomy.Off},
										remapping={'depth': 'depth'})

			# x:852 y:67
			OperatableStateMachine.add('translate',
										BigTranslateState(topic="puddles/move_distance"),
										transitions={'Success': 'align', 'Failure': 'failed'},
										autonomy={'Success': Autonomy.Off, 'Failure': Autonomy.Off},
										remapping={'x': 'x_start', 'y': 'y_start'})

			# x:761 y:208
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

			# x:364 y:24
			OperatableStateMachine.add('setPitch0State',
										BigPitchState(topic="/puddles/go_to_pitch"),
										transitions={'Success': 'setRoll0State', 'Failure': 'failed'},
										autonomy={'Success': Autonomy.Off, 'Failure': Autonomy.Off},
										remapping={'angle': 'initPitchAngle'})

			# x:575 y:55
			OperatableStateMachine.add('setRoll0State',
										BigRollState(topic="/puddles/go_to_roll"),
										transitions={'Success': 'setYaw0State', 'Failure': 'failed'},
										autonomy={'Success': Autonomy.Off, 'Failure': Autonomy.Off},
										remapping={'angle': 'initRollAngle'})

			# x:697 y:32
			OperatableStateMachine.add('setYaw0State',
										BigYawState(topic="/puddles/go_to_yaw"),
										transitions={'Success': 'translate', 'Failure': 'failed'},
										autonomy={'Success': Autonomy.Off, 'Failure': Autonomy.Off},
										remapping={'angle': 'initYawAngle'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
