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
from riptide_states.TransferToGlobal import TransferToGlobal
from riptide_states.big_move_state import BigMoveState
from riptide_states.big_roll_state import BigRollState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Feb 01 2021
@author: Toby McGuire
'''
class GateTaskSM(Behavior):
	'''
	does the gate task, better
	'''


	def __init__(self):
		super(GateTaskSM, self).__init__()
		self.name = 'GateTask'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:82 y:333, x:472 y:301
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.x = 0
		_state_machine.userdata.y = 0
		_state_machine.userdata.z = 0
		_state_machine.userdata.orientation = None
		_state_machine.userdata.rollAngle = 90

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:107 y:75
			OperatableStateMachine.add('GuessFrontOfGate',
										GetFrontOf(target="gate_frame"),
										transitions={'Success': 'Move'},
										autonomy={'Success': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'z': 'z', 'orientation': 'orientation'})

			# x:1061 y:117
			OperatableStateMachine.add('MakeGlobal I',
										TransferToGlobal(x=2, y=0, z=0, orientation=None),
										transitions={'Success': 'Move I'},
										autonomy={'Success': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'z': 'z', 'orientation': 'orientation'})

			# x:1051 y:440
			OperatableStateMachine.add('MakeGlobal II',
										TransferToGlobal(x=2, y=0, z=0, orientation=None),
										transitions={'Success': 'Move II'},
										autonomy={'Success': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'z': 'z', 'orientation': 'orientation'})

			# x:649 y:569
			OperatableStateMachine.add('MakeGlobal III',
										TransferToGlobal(x=1, y=0, z=0, orientation=None),
										transitions={'Success': 'Move III'},
										autonomy={'Success': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'z': 'z', 'orientation': 'orientation'})

			# x:304 y:560
			OperatableStateMachine.add('MakeGlobal IIII',
										TransferToGlobal(x=1, y=0, z=0, orientation=None),
										transitions={'Success': 'Move IIII'},
										autonomy={'Success': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'z': 'z', 'orientation': 'orientation'})

			# x:327 y:86
			OperatableStateMachine.add('Move',
										BigMoveState(),
										transitions={'done': 'Get Front of Gate', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'z': 'z', 'orientation': 'orientation'})

			# x:1086 y:217
			OperatableStateMachine.add('Move I',
										BigMoveState(),
										transitions={'done': 'Roll I', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'z': 'z', 'orientation': 'orientation'})

			# x:929 y:514
			OperatableStateMachine.add('Move II',
										BigMoveState(),
										transitions={'done': 'Roll II', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'z': 'z', 'orientation': 'orientation'})

			# x:542 y:577
			OperatableStateMachine.add('Move III',
										BigMoveState(),
										transitions={'done': 'Roll III', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'z': 'z', 'orientation': 'orientation'})

			# x:187 y:522
			OperatableStateMachine.add('Move IIII',
										BigMoveState(),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'z': 'z', 'orientation': 'orientation'})

			# x:914 y:100
			OperatableStateMachine.add('MoveToGate',
										BigMoveState(),
										transitions={'done': 'MakeGlobal I', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'z': 'z', 'orientation': 'orientation'})

			# x:1091 y:322
			OperatableStateMachine.add('Roll I',
										BigRollState(angle=180),
										transitions={'Success': 'MakeGlobal II', 'Failure': 'failed'},
										autonomy={'Success': Autonomy.Off, 'Failure': Autonomy.Off})

			# x:793 y:544
			OperatableStateMachine.add('Roll II',
										BigRollState(angle=90),
										transitions={'Success': 'MakeGlobal III', 'Failure': 'failed'},
										autonomy={'Success': Autonomy.Off, 'Failure': Autonomy.Off})

			# x:449 y:577
			OperatableStateMachine.add('Roll III',
										BigRollState(angle=90),
										transitions={'Success': 'MakeGlobal IIII', 'Failure': 'failed'},
										autonomy={'Success': Autonomy.Off, 'Failure': Autonomy.Off})

			# x:699 y:91
			OperatableStateMachine.add('Get Front of Gate',
										GetFrontOf(target="gate_frame"),
										transitions={'Success': 'MoveToGate'},
										autonomy={'Success': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'z': 'z', 'orientation': 'orientation'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
