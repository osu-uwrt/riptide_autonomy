/** 
 * Include file that contains all includes needed for autonomy
 */

#ifndef AUTONOMY_H
#define AUTONOMY_H

#include <iostream>
#include <functional>
#include <unistd.h>
#include <chrono>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer_client.h"
#include "rclcpp/rclcpp.hpp"

//define logger for RCLCPP_INFO, RCLCPP_WARN, and RCLCPP_ERROR
#define log rclcpp::get_logger("rclcpp")

//useful constant values for autonomy
const std::string
    STEADY_TOPIC = "steady",
    ODOMETRY_TOPIC = "odometry/filtered",
    POSITION_TOPIC = "position",
    ORIENTATION_TOPIC = "orientation",
    ANGULAR_VELOCITY_TOPIC = "angular_velocity";

/**
 * @brief A BT SyncActionNode with an init() method that takes a ROS node as a parameter.
 * This class should be inherited by every state used by the robot to avoid creating a new 
 * ros node every time a state change happens.
 */
class UWRTSyncActionNode : public BT::SyncActionNode {
    public:
    UWRTSyncActionNode(const std::string& name, const BT::NodeConfiguration& config)
     : BT::SyncActionNode(name, config) { };

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    virtual void init(rclcpp::Node::SharedPtr node) = 0;
};

#endif // AUTONOMY_H