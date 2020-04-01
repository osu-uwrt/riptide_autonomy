#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from riptide_autonomy.depthparameter_sm import DepthParameterSM
from riptide_states.big_align_state import BigAlignState
from riptide_states.big_distance_state import BigDistanceState
from riptide_states.big_translate_state import BigTranslateState
from riptide_states.big_gate_maneuver_state import BigGateManeuverState
from riptide_autonomy.yawbehavior_sm import YawBehaviorSM
from riptide_autonomy.rollbehavior_sm import RollBehaviorSM
from riptide_autonomy.pitchbehavior_sm import PitchBehaviorSM
from riptide_autonomy.translatebehavior_sm import TranslateBehaviorSM
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
		self.add_parameter('yaw_init', 0)
		self.add_parameter('depth_init', -3.3)
		self.add_parameter('x_start', 0)

		# references to used behaviors
		self.add_behavior(DepthParameterSM, 'DepthParameter')
		self.add_behavior(YawBehaviorSM, 'YawBehavior')
		self.add_behavior(RollBehaviorSM, 'RollBehavior')
		self.add_behavior(PitchBehaviorSM, 'PitchBehavior')
		self.add_behavior(TranslateBehaviorSM, 'TranslateBehavior')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:186 y:551, x:790 y:241
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.bboxWidth = 0.07
		_state_machine.userdata.hold = True
		_state_machine.userdata.obj = 'Gate'
		_state_machine.userdata.gateLeft = .45
		_state_machine.userdata.y_start = 0
		_state_machine.userdata.x_start = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:147 y:102
			OperatableStateMachine.add('DepthParameter',
										self.use_behavior(DepthParameterSM, 'DepthParameter',
											parameters={'depth': dep}),
										transitions={'finished': 'PitchBehavior', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:1003 y:232
			OperatableStateMachine.add('align',
										BigAlignState(topic="/puddles/align"),
										transitions={'Success': 'distance', 'Failure': 'failed'},
										autonomy={'Success': Autonomy.Off, 'Failure': Autonomy.Off},
										remapping={'obj': 'obj', 'bboxWidth': 'bboxWidth', 'hold': 'hold'})

			# x:1015 y:363
			OperatableStateMachine.add('distance',
										BigDistanceState(topic="/puddles/get_distance"),
										transitions={'Success': 'translateLeftofGate', 'Failure': 'failed'},
										autonomy={'Success': Autonomy.Off, 'Failure': Autonomy.Off},
										remapping={'object': 'obj', 'dist': 'dist'})

			# x:711 y:536
			OperatableStateMachine.add('translate2',
										BigTranslateState(topic="/puddles/move_distance"),
										transitions={'Success': 'gateManeuver', 'Failure': 'failed'},
										autonomy={'Success': Autonomy.Off, 'Failure': Autonomy.Off},
										remapping={'x': 'dist', 'y': 'y_start'})

			# x:424 y:502
			OperatableStateMachine.add('gateManeuver',
										BigGateManeuverState(topic="/puddles/gate_maneuver"),
										transitions={'Success': 'finished', 'Failure': 'failed'},
										autonomy={'Success': Autonomy.Off, 'Failure': Autonomy.Off})

			# x:1011 y:577
			OperatableStateMachine.add('translateLeftofGate',
										BigTranslateState(topic="/puddles/move_distance"),
										transitions={'Success': 'translate2', 'Failure': 'failed'},
										autonomy={'Success': Autonomy.Off, 'Failure': Autonomy.Off},
										remapping={'x': 'x_start', 'y': 'gateLeft'})

			# x:736 y:46
			OperatableStateMachine.add('YawBehavior',
										self.use_behavior(YawBehaviorSM, 'YawBehavior',
											parameters={'angle': ya}),
										transitions={'finished': 'TranslateBehavior', 'failed': 'TranslateBehavior'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:529 y:28
			OperatableStateMachine.add('RollBehavior',
										self.use_behavior(RollBehaviorSM, 'RollBehavior'),
										transitions={'finished': 'YawBehavior', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:311 y:40
			OperatableStateMachine.add('PitchBehavior',
										self.use_behavior(PitchBehaviorSM, 'PitchBehavior'),
										transitions={'finished': 'RollBehavior', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:984 y:70
			OperatableStateMachine.add('TranslateBehavior',
										self.use_behavior(TranslateBehaviorSM, 'TranslateBehavior',
											parameters={'x': x}),
										transitions={'finished': 'align', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
