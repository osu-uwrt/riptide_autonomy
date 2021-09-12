#ifndef STATES_H
#define STATES_H

/**
 * States (also called nodes) header file.
 */

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "moveit/moveit_cpp/moveit_cpp.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "ros/ros.h"

namespace states {
    class big_move_state : public BT::SyncActionNode {
        public:
        big_move_state(const std::string& name, const BT::NodeConfiguration& config)
         : BT::SyncActionNode(name, config) {
             init();
         };

        static BT::PortsList providedPorts();
        void init();
        void steadyCallback(const std_msgs::Bool::ConstPtr& msg);
        void locCallback(const nav_msgs::Odometry::ConstPtr& msg);
        BT::NodeStatus tick() override;

        private:
        bool onEnter();
        
        static const int CACHE_SIZE = 256;
        static const double threshold = 0.4;

        const std::string
            steadyTopic = "/puddles/steady",
            locTopic = "/puddles/odometry/filtered",
            groupName = "puddles_base";

        ros::Subscriber
            steadySubscriber,
            locSubscriber;

        bool hasEntered;
        double x, y, z;
        geometry_msgs::Quaternion orientation;

        std_msgs::Bool::ConstPtr latestSteadyMessage;
        nav_msgs::Odometry::ConstPtr latestLocMessage;

    };
}

#endif