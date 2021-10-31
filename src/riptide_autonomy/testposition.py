#! /usr/bin/env python
from riptide_autonomy.msg import *
import rospy
import actionlib

if __name__ == "__main__":
    rospy.init_node("go_to_position_client")
    client = actionlib.SimpleActionClient("Trajectory", TrajectoryAction)
    rospy.loginfo("waiting for server")
    client.wait_for_server()
    goal = TrajectoryGoal()
    goal.point.x = 0
    goal.point.y = 0
    goal.point.z = 0
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("done")
