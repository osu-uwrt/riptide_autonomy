#ifndef STATES_H
#define STATES_H

/**
 * States header file.
 */

#include <iostream>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer_client.h"
#include "moveit/moveit_cpp/moveit_cpp.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include <moveit/moveit_cpp/planning_component.h>
#include "ros/ros.h"
#include "ros/package.h"

namespace states {
    static const int CACHE_SIZE = 256;

    /**
     * Moves the robot to a certain position.
     */
    class big_move_state : public BT::SyncActionNode {
        public:
        big_move_state(const std::string& name, const BT::NodeConfiguration& config)
         : BT::SyncActionNode(name, config) {
             init();
         };

        static BT::PortsList providedPorts();
        BT::NodeStatus tick() override;

        private:
        void init();
        bool onEnter();
        void steadyCallback(const std_msgs::Bool::ConstPtr& msg);
        void locCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void publishGoalPose();
        geometry_msgs::Vector3 toRPY(geometry_msgs::Quaternion orientation);
        
        static constexpr double threshold = 0.4;

        const std::string
            steadyTopic = "steady",
            locTopic = "odometry/filtered",
            positionTopic = "position",
            orientationTopic = "orientation",
            angVelocityTopic = "angular_velocity";

        ros::Subscriber
            steadySubscriber,
            locSubscriber;

        ros::Publisher 
            positionPublisher,
            orientationPublisher,
            angVelocityPublisher;

        bool
            hasEntered,
            steady,
            locExists,
            publishPosition,
            publishOrientation,
            publishAngVelocity;

        geometry_msgs::Pose 
            latestLocData, //current position and orientation
            goal;

        geometry_msgs::Vector3 
            angVelocity,
            goalOrientationRPY;
 
        ros::NodeHandle n;

    };


    /**
     * Calculates the necessary pose to "flatten" the robot.
     */
    class flatten_calculation_state : public BT::SyncActionNode {
        public:
        flatten_calculation_state(const std::string& name, const BT::NodeConfiguration& config) 
         : BT::SyncActionNode(name, config) { 
             init();
        }
        
        static BT::PortsList providedPorts();
        BT::NodeStatus tick() override;

        private:
        void init();
        void locCallback(const nav_msgs::Odometry::ConstPtr& msg);

        const std::string
            locTopic = "odometry/filtered";

        ros::Subscriber locSubcriber;
        geometry_msgs::Pose latestLocData;
        bool locExists;

        ros::NodeHandle n;
    };

    /**
     * Transforms relative coordinates to global space.
     */
    class to_world_frame_state : public BT::SyncActionNode {
        public:
        to_world_frame_state(const std::string& name, const BT::NodeConfiguration& config)
         : BT::SyncActionNode(name, config) {
         }
        
        static BT::PortsList providedPorts();
        BT::NodeStatus tick() override;
    };

    /**
     * Drives the robot forward with a certain velocity for a certain time
     */
    class velocity_state : public BT::SyncActionNode {
        public:
        velocity_state(const std::string& name, const BT::NodeConfiguration& config)
         : BT::SyncActionNode(name, config) {
             init();
         }
        
        static BT::PortsList providedPorts();
        BT::NodeStatus tick() override;

        private:
        void init();

        const std::string velocityTopic = "linear_velocity";

        ros::Publisher velocityPublisher;
        
        geometry_msgs::Vector3 velocities;
        bool hasEntered;
        int startTime;

        ros::NodeHandle n;
    };

    class search_state : public BT::SyncActionNode {
        public:
        search_state(const std::string& name, const BT::NodeConfiguration& config)
         : BT::SyncActionNode(name, config) {
             init();
         }
        
        static BT::PortsList providedPorts();
        BT::NodeStatus tick() override;
        
        private:
        void init();
        void guessCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
        void locCallback(const nav_msgs::Odometry::ConstPtr& msg);
        double distance(geometry_msgs::Vector3 point1, geometry_msgs::Vector3 point2);

        const std::string
            positionTopic = "position",
            orientationTopic = "orientation",
            locTopic = "odometry/filtered";

        ros::Publisher positionPublisher;
        ros::Publisher orientationPublisher;
        ros::Subscriber guessSubscriber;
        ros::Subscriber locSubscriber;

        ros::NodeHandle n;
    };

    class torpedo_align_state : public BT::SyncActionNode {
        public:
        torpedo_align_state(const std::string& name, const BT::NodeConfiguration& config)
         : BT::SyncActionNode(name, config) {
             init();
         }

        static BT::PortsList providedPorts();
        BT::NodeStatus tick() override;

        private:
        void init();

        const std::string positionTopic = "position";
        ros::Publisher positionPublisher;

        ros::NodeHandle n;
    };
}

#endif