#pragma once

#include <chrono>
#include <string>
#include <functional>

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/buffer.h>

/**
 * 
 * OSU-UWRT autonomy library header; helpful includes, values, and functions are declared here
 * 
 */

using namespace std::chrono_literals;
using namespace std::placeholders;

//useful topic names for autonomy
const std::string
    ODOMETRY_TOPIC = "odometry/filtered",
    CONTROL_LINEAR_TOPIC = "controller/linear",
    CONTROL_ANGULAR_TOPIC = "controller/angular",
    ACTUATOR_BUSY_TOPIC = "state/actuator/busy",
    ACTUATOR_STATUS_TOPIC = "state/actuator/status",
    ROBOT_KILLED_TOPIC = "state/kill",
    ROBOT_AUX_TOPIC = "state/aux",
    LED_COMMAND_TOPIC = "command/led",
    DETECTIONS_TOPIC = "detected_objects";

//service names for autonomy
const std::string 
    CLAW_SERVER_NAME = "command/actuator/claw",
    DROPPER_SERVER_NAME = "command/actuator/dropper",
    TORPEDO_SERVER_NAME = "command/actuator/torpedo",
    MAPPING_SERVER_NAME = "mapping_target",
    SET_POSE_SERVER_NAME = "set_pose";


/**
 * 
 * TYPES
 *  
 */
inline std::pair<std::string, BT::PortInfo> UwrtInput(BT::StringView name, BT::StringView description = {}) {
    return BT::InputPort<std::string>(name, description);
}

inline std::pair<std::string, BT::PortInfo> UwrtOutput(BT::StringView name, BT::StringView description = {}) {
    return BT::OutputPort<std::string>(name, description);
}

/**
 *
 * UTILITY METHODS
 *
 */

/**
 * @brief Gets the value of an environment variable. Prints to the console and returns "" if no value could be found.
 * 
 * @param name The name of the environment variable to get.
 * @return std::string The value of the environment variable.
 */
std::string getEnvVar(const char *name);

/**
 * @brief Registers plugins needed to run UWRTs behavior trees or test nodes.
 * Using a register function also allows test suite to ensure that plugins being
 * tested are being loaded by the executor
 * 
 * @param factory The factory to register the plugins with.
 */
void registerPluginsForFactory(std::shared_ptr<BT::BehaviorTreeFactory> factory, const std::string& packageName);

/**
 * @brief Gives the passed ros context to each node in the behavior tree that needs it.
 * If this function is not called before running a tree, the tree will segfault if a 
 * node tries to access ROS.
 * 
 * @param tree The behavior tree to initialize
 * @param rosContext The ROS context to give to the tree
 */
void initRosForTree(BT::Tree& tree, rclcpp::Node::SharedPtr rosContext);

/**
 * @brief Transforms a pose by the provided transform
 * 
 * @param pose The pose to transform
 * @param transform The transform to apply to the pose
 * @return geometry_msgs::msg::Pose The pose after being transformed
 */
geometry_msgs::msg::Pose doTransform(geometry_msgs::msg::Pose pose, geometry_msgs::msg::TransformStamped transform);


bool lookupTransformNow(
    rclcpp::Node::SharedPtr node,
    const std::shared_ptr<const tf2_ros::Buffer> buffer,
    const std::string& fromFrame,
    const std::string& toFrame,
    geometry_msgs::msg::TransformStamped& transform,
    bool lookupNext = false);


#define DEF_THROTTLE_TIMER(name) double name = 0


bool lookupTransformThrottled(
    rclcpp::Node::SharedPtr node,
    const std::shared_ptr<const tf2_ros::Buffer> buffer,
    const std::string& fromFrame,
    const std::string& toFrame,
    double throttleDuration,
    double& lastLookup,
    geometry_msgs::msg::TransformStamped& transform,
    bool lookupNext = false);


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
 * @brief Get a thing from a BT blackboard.
 * 
 * @tparam T The type of the pointer to grab.
 * @param n The UwrtBtNode to get the bb value from
 * @param key The name of the value to grab.
 * @param value The variable to be populated with the desired blackboard entry.
 * @return true If the operation succeeds
 * @return false If the operation fails
 */
template<typename T>
bool getFromBlackboard(rclcpp::Node::SharedPtr rosnode, BT::Blackboard::Ptr bb, const std::string& key, T& value) {
    try {
        if(bb->get<T>(key, value)) {
            return true;
        }
    } catch (std::runtime_error& ex) {
        RCLCPP_ERROR(rosnode->get_logger(), "Error getting blackboard value named \"%s\": %s", key.c_str(), ex.what());
    }

    RCLCPP_ERROR(rosnode->get_logger(), "No blackboard value named \"%s\"", key.c_str());
    return false;
}
