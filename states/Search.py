#! /usr/bin/env python

import rospy
import smach

class Search(smach.State):
    """
    Tries to find a given object and aligns the robot to face it.

    @param search_object => String
        indictation of what to look for.
    @param args => Dictionary
        maps the required data for the search. See substates for details.
    """
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['Success', 'Failure'],
            input_keys=['search_object','args'])
    
    def execute(self, userdata):
        status = 'Success'
        sm = smach.StateMachine(outcomes=['Success', 'Failure'])
        sm.userdata.args = userdata.args
        

        return status
