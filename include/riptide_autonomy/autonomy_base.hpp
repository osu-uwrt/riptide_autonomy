#pragma once

#include <iostream>
#include <chrono>

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <ament_index_cpp/get_package_prefix.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <riptide_msgs2/msg/actuator_command.hpp>
#include <riptide_msgs2/msg/actuator_status.hpp>
#include <riptide_msgs2/msg/robot_state.hpp>
#include <riptide_msgs2/msg/controller_command.hpp>

#include <robot_localization/srv/set_pose.hpp>

#include <riptide_msgs2/action/align_torpedos.hpp>
#include <riptide_msgs2/action/actuate_torpedos.hpp>
#include <riptide_msgs2/action/change_claw_state.hpp>
#include <riptide_msgs2/action/actuate_droppers.hpp>


//define logger for RCLCPP_INFO, RCLCPP_WARN, and RCLCPP_ERROR
#define log rclcpp::get_logger("autonomy")
