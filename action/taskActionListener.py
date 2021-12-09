#!/usr/bin/env python3
import rospy
import roslaunch
import actionlib

import riptide_autonomy.msg

class TaskActionListenerAction(object):
    # create messages that are used to publish feedback/result
    _result = riptide_autonomy.msg.TaskActionListenerResult()

    process_started = False
    process_start_failed = False
    should_start_process = False

    def __init__(self, name):
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self.cli_args = ['riptide_autonomy', None, 'robot:=' + rospy.get_param("~robot")]

        rospy.logdebug("Starting TaskActionListener ActionServer on node %s", name)

        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, riptide_autonomy.msg.TaskActionListenerAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal: riptide_autonomy.msg.TaskActionListenerGoal):
        # helper variables
        r = rospy.Rate(1)
        preempted = False
        
        self.cli_args[1] = goal.autonomy_launchfile

        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(self.cli_args)[0]
        roslaunch_args = self.cli_args[2:]

        self.parent = roslaunch.parent.ROSLaunchParent(self.uuid, [(roslaunch_file, roslaunch_args)], sigint_timeout=8.0, sigterm_timeout=2.0)

        # publish info to the console for the user
        rospy.logdebug('%s: Executing launchfile %s with params %s' % (self._action_name, roslaunch_file, roslaunch_args))
        
        # Wait for main thread to start the process
        self.should_start_process = True
        while not self.process_started and not self.process_start_failed:
            r.sleep()
        self.process_started = False

        if self.process_start_failed:
            self.process_start_failed = False
            rospy.logdebug('%s: Aborted' % self._action_name)
            self._as.set_aborted()
            return

        # Execute action until process dies
        while self.parent.pm.is_alive():
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.logdebug('%s: Preempting...' % self._action_name)
                preempted = True
                break
            r.sleep()

        # Shutdown cleanly, and wait until fully shutdown if preempted
        self.parent.shutdown()
        while self.parent.pm.is_alive():
            r.sleep()
        
        
        if preempted:
            rospy.logdebug('%s: Preempted' % self._action_name)
            self._as.set_preempted()
        else:            
            if len(self.parent.pm.dead_list) != 1 and len(self.parent.pm.procs) != 0:
                rospy.logwarn("Expected only one process to be launched, %d found. Task success may be calculated wrong", len(self.parent.pm.dead_list) + len(self.parent.pm.procs))
                
            return_code = self.parent.pm.dead_list[0].exit_code

            if return_code == 0:
                rospy.logdebug('%s: Succeeded' % self._action_name)
                self._as.set_succeeded(self._result)
            else:
                # Note: A return code of None means the process died
                rospy.logdebug('%s: Failed, return code %s', self._action_name, str(return_code))
                self._as.set_aborted()

if __name__ == '__main__':
    rospy.init_node("task_action_listener")
    server = TaskActionListenerAction(rospy.get_name())
    
    # Hack required since parent.start() can only run on the main thread because it registers signals
    main_rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        if server.should_start_process:
            server.should_start_process = False
            try:
                server.parent.start()
                server.process_started = True
            except Exception as e:
                rospy.logerr("Error during task execution:", exc_info=e)
                server.process_start_failed = True
        main_rate.sleep()