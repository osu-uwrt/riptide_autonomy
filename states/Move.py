import rospy
import smach
from actionTools import *

class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['Success', 'Failure'],
            input_keys=['type','args'])
    
    def execute(self, userdata):
        if type == 'move':
            rospy.loginfo('Moving by vector <%f, %f>'%(userdata.args.x, userdata.args.y))
            moveAction(userdata.args.x, userdata.args.x).wait_for_result()
        elif type == 'depth':
            rospy.loginfo('Moving down %f'%userdata.args.depth)
            depthAction(userdata.args.depth)
        elif type == 'yaw':
            rospy.loginfo('Moving with angle %f'%userdata.args.angle)
            yawAction(userdata.args.angle)
        return 'Success'