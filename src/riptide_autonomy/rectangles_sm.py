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
Created on Wed Nov 11 2020
@author: Toby
'''
class RectanglesSM(Behavior):
	'''
	Heh
	'''


	def __init__(self):
		super(RectanglesSM, self).__init__()
		self.name = 'Rectangles'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], output_keys=['x', 'y'])
		_state_machine.userdata.x = 0
		_state_machine.userdata.y = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:228 y:134
			OperatableStateMachine.add('Rect',
										BigRectangleState(topic="/puddles/state/bboxes", threshold=.6, time=.2, target="cutie", timeout=2),
										transitions={'Success': 'finished', 'Failed': 'failed'},
										autonomy={'Success': Autonomy.Off, 'Failed': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
