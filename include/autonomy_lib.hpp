#ifndef AUTONOMY_LIB_H
#define AUTONOMY_LIB_H

/**
 * 
 * OSU-UWRT autonomy library header; helpful includes, values, and functions are declared here
 * 
 */

#include <iostream>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "riptide_msgs2/msg/actuator_command.hpp"
#include "riptide_msgs2/msg/actuator_status.hpp"
#include "riptide_msgs2/msg/robot_state.hpp"
#include "riptide_msgs2/msg/controller_command.hpp"

#include "robot_localization/srv/set_pose.hpp"

#include "riptide_msgs2/action/align_torpedos.hpp"

#include "UwrtBtNode.hpp"



//define logger for RCLCPP_INFO, RCLCPP_WARN, and RCLCPP_ERROR
#define log rclcpp::get_logger("autonomy")

//useful constant values for autonomy
const std::string
    ODOMETRY_TOPIC = "odometry/filtered",
    POSITION_TOPIC = "controller/linear",
    ORIENTATION_TOPIC = "controller/angular",
    ACTUATOR_COMMAND_TOPIC = "command/actuator",
    ACTUATOR_STATUS_TOPIC = "state/actuator",
    ROBOT_STATE_TOPIC = "state/robot";

/**
 *
 * UTILITY METHODS
 *
 */

/**
 * @brief Transforms a pose by the provided transform
 * 
 * @param pose The pose to transform
 * @param transform The transform to apply to the pose
 * @return geometry_msgs::msg::Pose The pose after being transformed
 */
geometry_msgs::msg::Pose doTransform(geometry_msgs::msg::Pose pose, geometry_msgs::msg::TransformStamped transform);

/**
 * @brief Looks up necessary transform between two poses the transforms specified coords into cords in the new pose
 * 
 * @param relative The pose in fromFrame to transform to toFrame.
 * @param fromFrame The current frame of the relative pose
 * @param toFrame The desired frame of the relative pose
 * @return geometry_msgs::msg::Pose The equlivalent pose in a new frame
 */
std::tuple<geometry_msgs::msg::Pose, bool> transformBetweenFrames(geometry_msgs::msg::Pose relative, std::string fromFrame, std::string toFrame, rclcpp::Node::SharedPtr rosnode);

/**
 * @brief Converts a quaternion to Euler (roll-pitch-yaw) angles in radians.
 *
 * @param quat The quaternion orientation to convert.
 * @return geometry_msgs::msg::Vector3 The orientation in roll pitch yaw.
 */
geometry_msgs::msg::Vector3 toRPY(geometry_msgs::msg::Quaternion quat);

/**
 * @brief Converts RPY to quaternion.
 *
 * @param rpy The Euler orientation to convert.
 * @return geometry_msgs::msg::Quaternion The quaternion represented by the Vector3
 */
geometry_msgs::msg::Quaternion toQuat(geometry_msgs::msg::Vector3 rpy);

/**
 * @brief Converts the passed point to a Vector3 message.
 *
 * @param pt The point to convert to Vector3.
 * @return geometry_msgs::msg::Vector3 A Vector3 message that is equal to the passed Point.
 */
geometry_msgs::msg::Vector3 pointToVector3(geometry_msgs::msg::Point pt);

/**
 * @brief Converts the passed Vector3 to a Point message.
 *
 * @param vec3 The Vector3 to convert to a point.
 * @return geometry_msgs::msg::Point A Point message that is equal to the passed Vector3.
 */
geometry_msgs::msg::Point vector3ToPoint(geometry_msgs::msg::Vector3 vec3);

/**
 * @brief Computes the length of a Vector3.
 *
 * @param vec3 The vector3 to measure length of.
 * @return double the length of the passed vector.
 */
double vector3Length(geometry_msgs::msg::Vector3 vec3);

/**
 * @brief Get a thing from the BT blackboard.
 *
 * @param treeNode The behaviortree node to grab the blackboard value from.
 * @param name The name of the blackboard value to get.
 * @param value Will be populated with the value grabbed from the blackboard.
 * @return true if the operation succeeded, false otherwise.
 */
template<typename T>
bool getFromBlackboard(BT::TreeNode& treeNode, std::string name, T* value);

/**
 * @brief Inserts blackboard entries, where applicable, into the given string.
 *
 * @param str The string to populate from the blackboard.
 * @param treeNode The behaviortree node to grab the blackboard values from.
 * @return std::string The string with the blackboard entries inserted
 */
std::string stringWithBlackboardEntries(std::string str, BT::TreeNode& treeNode);

/**
 * @brief Calculates the distance between two given points.
 *
 * @param pt1 The first point.
 * @param pt2 The second point.
 * @return double The distance between point1 and point2.
 */
double distance(geometry_msgs::msg::Point pt1, geometry_msgs::msg::Point pt2);

/**
 * @brief Calculates the distance between two given points.
 *
 * @param pt1 The first point.
 * @param pt2 The second point.
 * @return double The distance between point1 and point2.
 */
double distance(geometry_msgs::msg::Vector3 pt1, geometry_msgs::msg::Vector3 pt2);


/**
 * 
 * CLASSES 
 * 
 */

/**
 * @brief UWRT superclass for integrating SyncActionNodes with ROS. 
 * Uses multiple-inheritance to create a class that has the properties of 
 * both the SyncActionNode and the UWRT general node superclass.
 */
class UWRTSyncActionNode : public BT::SyncActionNode, public UwrtBtNode {
    public:
    UWRTSyncActionNode(const std::string& name, const BT::NodeConfiguration& config)
     : SyncActionNode(name, config) { };

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

/**
 * @brief UWRT superclass for integrating ConditionNodes with ROS.
 * Similar to UWRTSyncActionNode, this class inherits both the BT 
 * ConditionNode and UwrtBtNode.
 */
class UWRTConditionNode : public BT::ConditionNode, UwrtBtNode {
    public:
    UWRTConditionNode(const std::string& name, const BT::NodeConfiguration& config)
     : ConditionNode(name, config) { };
    
    static BT::PortsList providedPorts();
    BT::NodeStatus tick();
};

/**
 * @brief UWRT superclass for integrating DecoratorNodes with ROS.
 * Operates exactly the same as UWRTConditionNode and UWRTSyncActionNode.
 */
class UWRTDecoratorNode : public BT::DecoratorNode, UwrtBtNode {
    public:
    UWRTDecoratorNode(const std::string& name, const BT::NodeConfiguration& config)
     : DecoratorNode(name, config) { };
    
    static BT::PortsList providedPorts();
    BT::NodeStatus tick();
};

/**
 * @brief Basic states that do not require a ROS node.
 * These will be used for basic computations or prints to rclcpp.
 */
class SimpleActions {
    public:
    static void registerActions(BT::BehaviorTreeFactory *factory);
};

/**
 * @brief Basic conditions that do not require a ROS node.
 * Used for basic computations that can be checked without using a publisher or subscriber.
 *
 * TODO: move ActuatorStateCheckers here
 */
class SimpleConditions {
    public:
    static void registerConditions(BT::BehaviorTreeFactory *factory);
};

/**
 * @brief this class is about to go away real soon...
 */
class ActuatorConditions {
    public:
    //register states
    static void registerConditions(BT::BehaviorTreeFactory *factory);
};

#endif // AUTONOMY_LIB_H
