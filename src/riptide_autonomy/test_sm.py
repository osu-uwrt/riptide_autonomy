#!/usr/bin/env python3
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from riptide_states.big_rectangle_state import BigRectangleState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Oct 18 2020
@author: The big dogs
'''
class TestSM(Behavior):
	'''
	big dogs
	'''


	def __init__(self):
		super(TestSM, self).__init__()
		self.name = 'Test'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:131 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], output_keys=['xout', 'yout'])
		_state_machine.userdata.xout = 0
		_state_machine.userdata.yout = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:514 y:87
			OperatableStateMachine.add('Rectangle',
										BigRectangleState(topic=/puddles/state/bboxes, threshold=.65, time=3, target=cutie, timeout=5),
										transitions={'Success': 'finished', 'Failed': 'failed'},
										autonomy={'Success': Autonomy.Off, 'Failed': Autonomy.Off},
										remapping={'x': 'xout', 'y': 'yout'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
