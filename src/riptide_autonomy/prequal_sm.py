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
from riptide_states.position_parameter_state import PositionParameterState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Mar 16 2021
@author: Parth Parekh
'''
class prequalSM(Behavior):
	'''
	Prequalification task
	'''


	def __init__(self):
		super(prequalSM, self).__init__()
		self.name = 'prequal'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:66 y:558, x:589 y:308
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.x = 0
		_state_machine.userdata.y = 0
		_state_machine.userdata.z = 0
		_state_machine.userdata.orientation = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:65 y:81
			OperatableStateMachine.add('Make Global I',
										TransferToGlobal(x=0, y=0, z=-2, orientation=None),
										transitions={'Success': 'Initial Descent'},
										autonomy={'Success': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'z': 'z', 'orientation': 'orientation'})

			# x:1122 y:398
			OperatableStateMachine.add('Forwards',
										TransferToGlobal(x=4, y=0, z=0, orientation=None),
										transitions={'Success': 'Move Fwds'},
										autonomy={'Success': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'z': 'z', 'orientation': 'orientation'})

			# x:291 y:78
			OperatableStateMachine.add('Initial Descent',
										BigMoveState(),
										transitions={'done': 'Make Global II', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'z': 'z', 'orientation': 'orientation'})

			# x:507 y:75
			OperatableStateMachine.add('Make Global II',
										TransferToGlobal(x=5, y=0, z=0, orientation=None),
										transitions={'Success': 'Move forwards towards the Pole'},
										autonomy={'Success': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'z': 'z', 'orientation': 'orientation'})

			# x:1059 y:506
			OperatableStateMachine.add('Move Fwds',
										BigMoveState(),
										transitions={'done': 'Translate Right', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'z': 'z', 'orientation': 'orientation'})

			# x:993 y:288
			OperatableStateMachine.add('Move Left',
										BigMoveState(),
										transitions={'done': 'Forwards', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'z': 'z', 'orientation': 'orientation'})

			# x:781 y:576
			OperatableStateMachine.add('Move Right',
										BigMoveState(),
										transitions={'done': 'Backwards', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'z': 'z', 'orientation': 'orientation'})

			# x:94 y:284
			OperatableStateMachine.add('Move back to origin',
										BigMoveState(),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'z': 'z', 'orientation': 'orientation'})

			# x:323 y:485
			OperatableStateMachine.add('Move backwards',
										BigMoveState(),
										transitions={'done': 'go back to abs 0', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'z': 'z', 'orientation': 'orientation'})

			# x:698 y:80
			OperatableStateMachine.add('Move forwards towards the Pole',
										BigMoveState(),
										transitions={'done': 'PoleFinder', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'z': 'z', 'orientation': 'orientation'})

			# x:1149 y:113
			OperatableStateMachine.add('Move to The Pole',
										BigMoveState(),
										transitions={'done': 'Translate left', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'z': 'z', 'orientation': 'orientation'})

			# x:958 y:86
			OperatableStateMachine.add('PoleFinder',
										GetFrontOf(target="pole_frame"),
										transitions={'Success': 'Move to The Pole'},
										autonomy={'Success': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'z': 'z', 'orientation': 'orientation'})

			# x:952 y:637
			OperatableStateMachine.add('Translate Right',
										TransferToGlobal(x=0, y=6, z=0, orientation=None),
										transitions={'Success': 'Move Right'},
										autonomy={'Success': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'z': 'z', 'orientation': 'orientation'})

			# x:1036 y:185
			OperatableStateMachine.add('Translate left',
										TransferToGlobal(x=0, y=-2, z=0, orientation=None),
										transitions={'Success': 'Move Left'},
										autonomy={'Success': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'z': 'z', 'orientation': 'orientation'})

			# x:177 y:402
			OperatableStateMachine.add('go back to abs 0',
										PositionParameterState(x=0, y=0, z=-1),
										transitions={'Success': 'Move back to origin', 'Failure': 'failed'},
										autonomy={'Success': Autonomy.Off, 'Failure': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'z': 'z', 'orientation': 'orientation'})

			# x:571 y:554
			OperatableStateMachine.add('Backwards',
										TransferToGlobal(x=-4, y=0, z=0, orientation=None),
										transitions={'Success': 'Move backwards'},
										autonomy={'Success': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'z': 'z', 'orientation': 'orientation'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
