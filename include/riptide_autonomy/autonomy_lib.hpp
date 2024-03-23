#ifndef AUTONOMY_LIB_H
#define AUTONOMY_LIB_H

/**
 * 
 * OSU-UWRT autonomy library header; helpful includes, values, and functions are declared here
 * 
 */

#include "riptide_autonomy/autonomy_base.hpp"
#include "riptide_autonomy/UwrtBtNode.hpp"

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
    MAPPING_SERVER_NAME = "mapping_target";


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

    return false;
}

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
bool getFromBlackboard(UwrtBtNode *n, const std::string& key, T& value) {
    if(!n->treeNode()->config().blackboard) {
        RCLCPP_ERROR(n->rosNode()->get_logger(), "Cannot get from blackboard! The passed TreeNode does not have one!");
        return false;
    }

    return getFromBlackboard<T>(n->rosNode(), n->treeNode()->config().blackboard, key, value);
}


template<typename T>
void postOutput(UwrtBtNode *n, const std::string& key, T value) {
    std::ostringstream stream;
    stream << value;

    n->treeNode()->setOutput(key, stream.str());
}

/**
 * @brief Attempts to get a port input from a TreeNode.
 * 
 * @tparam T The type of the port value to get.
 * @param n The node to get the port value of.
 * @param key The key of the port.
 * @param defaultValue The value to return if the key does not have a value
 * @param warnIfUndefined True if a warning should be printed if the key doesn't exist
 * @return T The value of the port or the default if there is none.
 */
template<typename T> 
T tryGetInput(UwrtBtNode *n, const std::string& key, const T defaultValue, bool warnIfUndefined) {
    auto op = n->treeNode()->getInput<std::string>(key);
    if(op.has_value()) {
        return BT::convertFromString<T>(op.value());
    } else if(warnIfUndefined) {
        RCLCPP_WARN(n->rosNode()->get_logger(), "Node %s does not have a value for required port with name %s!", n->treeNode()->name().c_str(), key.c_str());
    }

    return defaultValue;
}

/**
 * @brief Gets an input from a required port. A warning will be printed if the value does not exist
 * 
 * @tparam T The type of the port value.
 * @param n The TreeNode to get the value from
 * @param key The name of the value to get
 * @param defaultValue The value to return if the key does not have a value.
 * @return T The value of the port, or the default value if there is no such value.
 */
template<typename T>
T tryGetRequiredInput(UwrtBtNode *n, const std::string& key, const T defaultValue) {
    return tryGetInput<T>(n, key, defaultValue, true);
}

/**
 * @brief Attempts to get an optional value from a port.
 * 
 * @tparam T The type of the value to get.
 * @param n The TreeNode to get the port value from
 * @param key The name of the value to get.
 * @param defaultValue the value fto retuurn if the key does not have a value.
 * @return T The value of the port, or the default value if there is no such value.
 */
template<typename T>
T tryGetOptionalInput(UwrtBtNode *n, const std::string& key, const T defaultValue) {
    return tryGetInput<T>(n, key, defaultValue, false);
}

/**
 * @brief Attempts to get an entry from a BT blackboard, and returns a specified default value if the operation fails.
 * 
 * @tparam T The type of value to get.
 * @param blackboard The blackboard to use.
 * @param key The key to grab from the blackboard.
 * @param defaultValue The value to return if the operation fails
 * @return T The value of the blackboard entry, or the default value if it cannot be retrieved.
 */
template<typename T>
T getFromBlackboardWithDefault(BT::Blackboard::Ptr blackboard, const std::string& key, const T defaultValue) {
    T retval = defaultValue;
    getFromBlackboard(blackboard, key, retval);
    return retval;
}

/**
 * @brief Attempts to get an entry from a BT blackboard, and returns a specified default value if the operation fails.
 * 
 * @tparam T The type of value to get.
 * @param n The tree node whose blackboard to use.
 * @param key The key to grab from the blackboard.
 * @param defaultValue The value to return if the operation fails.
 * @return T The value of the blackboard entry, or the default value if it cannot be retrieved.
 */
template<typename T>
T getFromBlackboardWithDefault(UwrtBtNode& n, const std::string& key, T& defaultValue) {
    T retval = defaultValue;
    getFromBlackboard(n, key, retval);
    return retval;
}

/**
 * @brief Inserts blackboard entries, where applicable, into the given string.
 *
 * @param str The string to populate from the blackboard.
 * @param treeNode The behaviortree node to grab the blackboard values from.
 * @return std::string The string with the blackboard entries inserted
 */
std::string formatStringWithBlackboard(const std::string& str, UwrtBtNode *treeNode);

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

#endif // AUTONOMY_LIB_H
