#ifndef AUTONOMY_LIB_H
#define AUTONOMY_LIB_H

/**
 * 
 * OSU-UWRT autonomy library header; helpful includes, values, and functions are declared here
 * 
 */

#include "riptide_autonomy/autonomy_base.hpp"
#include "riptide_autonomy/UwrtBtNode.hpp"

//useful topic names for autonomy
const std::string
    ODOMETRY_TOPIC = "odometry/filtered",
    POSITION_TOPIC = "controller/linear",
    ORIENTATION_TOPIC = "controller/angular",
    ACTUATOR_COMMAND_TOPIC = "command/actuator",
    ACTUATOR_STATUS_TOPIC = "state/actuator",
    ROBOT_KILLED_TOPIC = "state/kill",
    ROBOT_AUX_TOPIC = "state/aux",
    LED_COMMAND_TOPIC = "command/led";

//action server names for autonomy
const std::string 
    CLAW_SERVER_NAME = "actuate_claw",
    DROPPER_SERVER_NAME = "actuate_droppers",
    TORPEDO_SERVER_NAME = "fire_torpedos",
    ARMER_SERVER_NAME = "arm_torpedo_dropper";

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
void registerPluginsForFactory(std::shared_ptr<BT::BehaviorTreeFactory> factory, const std::string packageName);

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

/**
 * @brief Looks up necessary transform between two poses then transforms specified coords into cords in the new pose. Use this overload if you have or can create a TF buffer
 * 
 * @param relative The pose in fromFrame to transform to toFrame.
 * @param fromFrame The current frame of the relative pose
 * @param toFrame The desired frame of the relative pose
 * @param rosnode The ROS node handle to use.
 * @param buffer The TF buffer to use.
 * @return std::tuple<geometry_msgs::msg::Pose, bool> The equlivalent pose in a new frame
 */
bool transformBetweenFrames(rclcpp::Node::SharedPtr rosnode, std::shared_ptr<tf2_ros::Buffer> buffer, geometry_msgs::msg::Pose original, std::string fromFrame, std::string toFrame, geometry_msgs::msg::Pose& result);

/**
 * @brief Looks up necessary transform between two poses then transforms specified coords into coords in the new pose. this overload will create a TF buffer and listener to use.
 * 
 * @param relative The pose in fromFrame to transform to toFrame
 * @param fromFrame The current frame of the relative pose
 * @param toFrame The desired frame of the relative pose
 * @param rosnode The ROS node handle to use.
 * @return std::tuple<geometry_msgs::msg::Pose, bool> The equivalent pose in a new frame
 */
bool transformBetweenFrames(rclcpp::Node::SharedPtr rosnode, geometry_msgs::msg::Pose relative, std::string fromFrame, std::string toFrame, geometry_msgs::msg::Pose& result);

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
 * @param blackboard The blackboard to grab from.
 * @param key The name of the value to grab.
 * @param value The variable to be populated with the desired blackboard entry.
 * @return true If the operation succeeds
 * @return false If the operation fails
 */
template<typename T>
bool getFromBlackboard(BT::Blackboard::Ptr blackboard, std::string key, T& value) {
    try {
        if(blackboard->get<T>(key, value)) {
            return true;
        }
    } catch (std::runtime_error const&) {
        RCLCPP_ERROR(log, "Could not cast blackboard entry \"%s\" to the correct type.", key.c_str());
    }

    return false;
}

/**
 * @brief Get a thing from the BT blackboard.
 *
 * @tparam T The type of the pointer to grab.
 * @param treeNode The behaviortree node to grab the blackboard value from.
 * @param name The name of the blackboard value to get.
 * @param value Will be populated with the value grabbed from the blackboard.
 * @return true if the operation succeeded, false otherwise.
 */
template<typename T>
bool getFromBlackboard(BT::TreeNode& n, std::string key, T& value) {
    if(!n.config().blackboard) {
        RCLCPP_ERROR(log, "Cannot get from blackboard! The passed TreeNode does not have one!");
        return false;
    }

    return getFromBlackboard(n.config().blackboard, key, value);
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
T tryGetInput(const BT::TreeNode *n, const std::string& key, const T defaultValue, bool warnIfUndefined) {
    auto op = n->getInput<T>(key);
    if(op.has_value()) {
        return op.value();
    } else if(warnIfUndefined) {
        RCLCPP_WARN(log, "Node %s does not have a value for required port with name %s!", n->name().c_str(), key.c_str());
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
T tryGetRequiredInput(const BT::TreeNode *n, const std::string& key, const T defaultValue) {
    return tryGetInput(n, key, defaultValue, true);
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
T tryGetOptionalInput(const BT::TreeNode *n, const std::string& key, const T defaultValue) {
    return tryGetInput(n, key, defaultValue, false);
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
T getFromBlackboardWithDefault(BT::Blackboard::Ptr blackboard, std::string key, const T defaultValue) {
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
T getFromBlackboardWithDefault(BT::TreeNode& n, std::string key, T& defaultValue) {
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
 * @brief UWRT superclass for BT action nodes
 */
class UWRTActionNode : public BT::StatefulActionNode, public UwrtBtNode {
    public:
    UWRTActionNode(const std::string& name, const BT::NodeConfiguration& config)
     : StatefulActionNode(name, config) { };

    // static BT::PortsList providedPorts();
};

/**
 * @brief UWRT superclass for integrating ConditionNodes with ROS.
 * Similar to UWRTSyncActionNode, this class inherits both the BT 
 * ConditionNode and UwrtBtNode.
 */
class UWRTConditionNode : public BT::ConditionNode, public UwrtBtNode {
    public:
    UWRTConditionNode(const std::string& name, const BT::NodeConfiguration& config)
     : ConditionNode(name, config) { };
    
    // static BT::PortsList providedPorts();
};

/**
 * @brief UWRT superclass for integrating DecoratorNodes with ROS.
 * Operates exactly the same as UWRTConditionNode and UWRTSyncActionNode.
 */
class UWRTDecoratorNode : public BT::DecoratorNode, public UwrtBtNode {
    public:
    UWRTDecoratorNode(const std::string& name, const BT::NodeConfiguration& config)
     : DecoratorNode(name, config) { };
    
    // static BT::PortsList providedPorts();
};

#endif // AUTONOMY_LIB_H
