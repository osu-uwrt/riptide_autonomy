#!/usr/bin/env python
import riptide_controllers.msg
import riptide_autonomy.msg
import actionlib
from flexbe_core import EventState, Logger
import rospy
import smach

from flexbe_core.proxy import ProxyPublisher, ProxyActionClient
from geometry_msgs.msg import PoseStamped


class RiptideMoveState(EventState):
	"""
	State to control the robot's movement

	-- topic 		string 			Topic to which the pose will be published.

	># pose			PoseStamped		Pose to be published.

	<= done							Pose has been published.

	"""
	
	def __init__(self, topic):
		"""Constructor"""
		super(RiptideMoveState, self).__init__(outcomes=['Success', 'Failure'],
												input_keys=['type','args'])

		self._topic = topic
		self._pub = ProxyPublisher({self._topic: PoseStamped})


	def execute(self, userdata):
		status = 'Success'
 		sm = smach.StateMachine(outcomes=['Success', 'Failure'])
 		sm.userdata.args = userdata.args
 		if userdata.type == 'translate':
 			with sm:
 				smach.StateMachine.add('TRANSLATION', Translate(),
 					transitions={'Success': 'Success', 'Failure' : 'Failure'},
 					remapping={'args':'args'})
 				status = sm.execute()
 		elif userdata.type == 'lqr':
 			#TODO Implement LQR
 			Logger.loginfo('ERROR: LQR is not implemented yet')
 			status = 'Failure'
 		elif userdata.type == 'depth':
 			with sm:
 				smach.StateMachine.add('DEPTH', Depth(),
 					transitions={'Success': 'Success', 'Failure' : 'Failure'},
 					remapping={'args':'args'})
 				status = sm.execute()
 		elif userdata.type == 'yaw':
 			with sm:
 				smach.StateMachine.add('YAW', Yaw(),
 					transitions={'Success': 'Success', 'Failure' : 'Failure'},
 					remapping={'args':'args'})
 				status = sm.execute()
 		elif userdata.type == 'pitch':
 			with sm:
 				smach.StateMachine.add('PITCH', Pitch(),
 					transitions={'Success': 'Success', 'Failure' : 'Failure'},
 					remapping={'args':'args'})
 				status = sm.execute()
 		elif userdata.type == 'roll':
 			with sm:
 				smach.StateMachine.add('ROLL', Roll(),
 					transitions={'Success': 'Success', 'Failure' : 'Failure'},
 					remapping={'args':'args'})
 				status = sm.execute()
 		elif userdata.type == 'gateManuever':
 			with sm:
 				smach.StateMachine.add('GATEMANUEVER', GateManuever(),
 					transitions={'Success':'Success', 'Failure':'Failure'})
 				status = sm.execute()
 		else:
 			status = "Failure"
			Logger.loginfo('ERROR: type of %s not recognized'%userdata.type)
 		return status
	
	def on_enter(self, userdata):
		self._pub.publish(self._topic, userdata.pose)


class Depth(EventState):
    """
    Handles moving the robot to a given depth in meters.

    @param depth => float
        the depth in meters the robot is aiming for
        
    """
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['Success', 'Failure'],
            input_keys=['args'])
        self.client = ProxyActionClient({"go_to_depth": riptide_controllers.msg.GoToDepthAction})
        
    def on_enter(self, userdata):
        Logger.loginfo('Moving to depth %f'%userdata.args['depth'])
        
        #self.client.wait_for_server()
        self.client.send_goal("go_to_depth", riptide_controllers.msg.GoToDepthGoal(userdata.args['depth']))

    def execute(self, userdata):
        if self.client.has_result("go_to_depth"):
            result = self.client.get_result("go_to_depth")
            status = 'Success'       
            return status

class GateManuever(EventState):
    """
    Fany way of getting through the gate
    """
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['Success', 'Failure'])
        self.topic = "gate_maneuver"
        self.client = ProxyActionClient({
            self.topic:riptide_controllers.msg.GateManeuverAction})
        #self.client.wait_for_server()
        
    def on_enter(self, userdata):
        self.client.send_goal(self.topic, riptide_controllers.msg.GateManeuverGoal())

    def execute(self, userdata):
        if self.client.has_result(self.topic):
            result = self.client.get_result(self.topic)
            status = 'Success'       
            return status

class Pitch(EventState):
    """
    Handles rotating the robot's pitch.
    Pitch is like tilting up/down.

    @param angle => float
        the angle to rotate by
        
    """
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['Success', 'Failure'],
            input_keys=['args'])
        self.topic = "go_to_pitch"
        self.client = ProxyActionClient({
            self.topic:riptide_controllers.msg.GoToPitchAction})
        #self.client.wait_for_server()

    def on_enter(self, userdata):
        self.client.send_goal(self.topic, riptide_controllers.msg.GoToPitchGoal(userdata.args['angle']))
        Logger.loginfo('Pitching with angle %f'%userdata.args['angle'])

    def execute(self, userdata):
       if self.client.has_result(self.topic):
            result = self.client.get_result(self.topic)
            status = 'Success'       
            return status

class Roll(EventState):
    """
    Handles rotating the robot's roll.
    Roll is like tilting the side.

    @param angle => float
        the angle to rotate by
        
    """
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['Success', 'Failure'],
            input_keys=['args'])
        self.topic = "go_to_roll"
        self.client = ProxyActionClient({
            self.topic:riptide_controllers.msg.GoToRollAction})
        #self.client.wait_for_server()

    
    def on_enter(self,userdata):
        Logger.loginfo('Rolling with angle %f'%userdata.args['angle'])
        self.client.send_goal(self.topic, riptide_controllers.msg.GoToRollGoal(userdata.args['angle']))

    def execute(self, userdata):
        if self.client.has_result(self.topic):
            result = self.client.get_result(self.topic)
            status = 'Success'       
            return status

class Translate(EventState):
    """
    Handles simple translations of the robot across the xy plane of the robot
    oriented with the reference frame of it being upright.

    @param x => float
        the distance to move in the x direction
    @param y => float
        the distance to move in the y direction

    """
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['Success', 'Failure'],
            input_keys=['args'])
        self.topic = "move_distance"
        self.client = ProxyActionClient({
            self.topic:riptide_controllers.msg.MoveDistanceAction})
        

    def on_enter(self, userdata):
        Logger.loginfo('Translating by vector <%f, %f>'%(userdata.args['x'], userdata.args['y']))
        self.client.send_goal(self.topic, riptide_controllers.msg.MoveDistanceGoal(userdata.args['x'], userdata.args['y']))
    def execute(self, userdata):
         if self.client.has_result(self.topic):
            result = self.client.get_result(self.topic)
            status = 'Success'       
            return status

class Yaw(EventState):
    """
    Handles rotating the robot's yaw by a given angle.
    Yaw is like left/right movement.

    @param angle => float
        the angle to rotate by
        
    """
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['Success', 'Failure'],
            input_keys=['args'])
        self.topic = "go_to_yaw"
        self.client = ProxyActionClient({
            self.topic, riptide_controllers.msg.GoToYawAction})
        

    def on_enter(self, userdata):
        Logger.loginfo('Yawing with angle %f'%userdata.args['angle'])
        self.client.send_goal(self.topic, riptide_controllers.msg.GoToYawGoal(userdata.args['angle']))
    
    def execute(self, userdata):
        if self.client.has_result(self.topic):
            result = self.client.get_result(self.topic)
            status = 'Success'       
            return status