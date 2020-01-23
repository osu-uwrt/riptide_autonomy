import rospy
import smach
from actionTools import *

class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['Success', 'Failure'],
            input_keys=['x', 'y'])
    
    def execute(self, userdata):
        rospy.loginfo('Moving by vector <%f, %f>'%(userdata.x, userdata.y))
        moveAction(userdata.x, userdata.y).wait_for_result()
        rospy.loginfo('I moved uwu')
        return 'Success'