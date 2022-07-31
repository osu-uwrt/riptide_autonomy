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
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "riptide_msgs2/msg/actuator_command.hpp"
#include "riptide_msgs2/msg/actuator_status.hpp"
#include "riptide_msgs2/msg/controller_command.hpp"
#include "riptide_msgs2/msg/robot_state.hpp"
#include "riptide_msgs2/action/align_torpedos.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_msgs/action/lookup_transform.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_localization/srv/set_pose.hpp"

using namespace BT;

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
 * @brief Transforms a pose.
 *
 * @return geometry_msgs::msg::Pose The original pose with the transform applied.
 */
geometry_msgs::msg::Pose doTransform(geometry_msgs::msg::Pose, geometry_msgs::msg::TransformStamped);

/**
 * @brief Converts a quaternion to Euler (roll-pitch-yaw) angles in radians.
 *
 * @return geometry_msgs::msg::Vector3 The orientation in roll pitch yaw.
 */
geometry_msgs::msg::Vector3 toRPY(geometry_msgs::msg::Quaternion);

/**
 * @brief Converts RPY to quaternion.
 *
 * @return geometry_msgs::msg::Quaternion The quaternion represented by the Vector3
 */
geometry_msgs::msg::Quaternion toQuat(geometry_msgs::msg::Vector3);

/**
 * @brief Converts the passed point to a Vector3 message.
 *
 * @return geometry_msgs::msg::Vector3 A Vector3 message that is equal to the passed Point.
 */
geometry_msgs::msg::Vector3 pointToVector3(geometry_msgs::msg::Point);

/**
 * @brief Converts the passed Vector3 to a Point message.
 *
 * @return geometry_msgs::msg::Point A Point message that is equal to the passed Vector3.
 */
geometry_msgs::msg::Point vector3ToPoint(geometry_msgs::msg::Vector3);

/**
 * @brief Computes the length of a Vector3.
 *
 * @return double the length of the passed vector.
 */
double vector3Length(geometry_msgs::msg::Vector3);

/**
 * @brief Get a thing from the BT blackboard.
 *
 * @return true if the operation succeeded, false otherwise.
 */
template<typename T>
bool getFromBlackboard(BT::TreeNode&, std::string, T*);

/**
 * @brief Inserts blackboard entries, where applicable, into the given string.
 *
 * @return std::string The string with the blackboard entries inserted
 */
std::string stringWithBlackboardEntries(std::string, BT::TreeNode&);

/**
 * @brief Calculates the distance between two given points.
 *
 * @return double The distance between point1 and point2.
 */
double distance(geometry_msgs::msg::Point, geometry_msgs::msg::Point);
double distance(geometry_msgs::msg::Vector3, geometry_msgs::msg::Vector3);

/**
 * STATES
 */

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
 * @brief A BT SyncActionNode with an init() method that takes a ROS node as a parameter.
 * This class should be inherited by every state used by the robot to avoid creating a new
 * ros node every time a state change happens.
 */
class UWRTSyncActionNode : public SyncActionNode {
    public:
    UWRTSyncActionNode(const std::string& name, const NodeConfiguration& config)
     : SyncActionNode(name, config) { };

    static PortsList providedPorts();
    NodeStatus tick() override;
    virtual void init(rclcpp::Node::SharedPtr node) = 0;
};

/**
 * @brief Class for a control node that retries its child until it times out or is successful.
 */
class RetryUntilSuccessfulOrTimeout : public BT::DecoratorNode {
    public:
    RetryUntilSuccessfulOrTimeout(const std::string& name, const NodeConfiguration& config)
     : DecoratorNode(name, config) { };

    static PortsList providedPorts() {
        return {
            InputPort<double>("num_seconds")
        };
    }

    protected:
    NodeStatus tick() override;
};

/**
 * @brief The base state template that you can copy and modify to create other states
 *
 * When creating a new state, copy this template and rename the class and the constructor to
 * the name of the state you are creating. For example, if you are writing a state that
 * aligns the robot to a thing, you might rename the class and constructor to "AlignState".
 *
 * After renaming the class, be sure to define the providedPorts() method. Any input/output
 * ports should be defined in this header to reduce verbosity in your source file.
 *
 * After defining your ports, delete this comment and replace it with a description of your state.
 *
 * DO NOT USE THIS CLASS OR ANY OF ITS METHODS AS A STATE. YOU WILL GET UNDEFINED REFERENCES
 * BECAUSE THIS CLASS DOESN'T GET COMPILED.
 *
 * TODO: Delete this comment and resolve other todos.
 */
class BaseState : public UWRTSyncActionNode { //TODO: Rename class to whatever your state is named.
    public:
    BaseState(const std::string& name, const NodeConfiguration& config) //TODO: Rename constructor to match class name.
     : UWRTSyncActionNode(name, config) { }

    /**
     * @brief Declares ports needed by this state.
     * @return PortsList Needed ports.
     */
    static PortsList providedPorts() {
        return {
            //TODO: define needed ports here...
        };
    }

    /**
     * @brief Initializes the node.
     * @param node The ROS node belonging to the current process.
     */
    void init(rclcpp::Node::SharedPtr) override;

    /**
     * @brief Executes the node.
     * This method will be called once by the tree and can block for as long
     * as it needs for the action to be completed. When execution completes,
     * this method must return either SUCCESS or FAILURE; it CANNOT return
     * IDLE or RUNNING.
     *
     * @return NodeStatus The result of the execution; SUCCESS or FAILURE.
     */
    NodeStatus tick() override;

    private:
    //process node
    rclcpp::Node::SharedPtr rosnode;
};


/**
 * @brief Publishes a message to the controller to move the robot.
 */
class PublishToController : public UWRTSyncActionNode { //TODO: Rename class to whatever your state is named.
    public:
    PublishToController(const std::string& name, const NodeConfiguration& config) //TODO: Rename constructor to match class name.
     : UWRTSyncActionNode(name, config) { }

    /**
     * @brief Declares ports needed by this state.
     * @return PortsList Needed ports.
     */
    static PortsList providedPorts() {
        return {
            InputPort<bool>("isOrientation"),
            InputPort<int>("mode"),
            InputPort<double>("x"),
            InputPort<double>("y"),
            InputPort<double>("z"),
        };
    }

    /**
     * @brief Initializes the node.
     * @param node The ROS node belonging to the current process.
     */
    void init(rclcpp::Node::SharedPtr) override;

    /**
     * @brief Executes the node
     *
     * @return NodeStatus The result of the execution; SUCCESS or FAILURE.
     */
    NodeStatus tick() override;

    private:
    //process node
    rclcpp::Node::SharedPtr rosnode;

    //publisher to controller
    rclcpp::Publisher<riptide_msgs2::msg::ControllerCommand>::SharedPtr
        positionPub,
        orientationPub;
};

/**
 * @brief Gets the odometry of the robot.
 */
class GetOdometry : public UWRTSyncActionNode { //TODO: Rename class to whatever your state is named.
    public:
    GetOdometry(const std::string& name, const NodeConfiguration& config) //TODO: Rename constructor to match class name.
     : UWRTSyncActionNode(name, config) { }

    /**
     * @brief Declares ports needed by this state.
     * @return PortsList Needed ports.
     */
    static PortsList providedPorts() {
        return {
            OutputPort<double>("x"),
            OutputPort<double>("y"),
            OutputPort<double>("z"),
            OutputPort<double>("or"),
            OutputPort<double>("op"),
            OutputPort<double>("oy")
        };
    }

    /**
     * @brief Initializes the node.
     * @param node The ROS node belonging to the current process.
     */
    void init(rclcpp::Node::SharedPtr) override;

    /**
     * @brief Executes the node.
     *
     * @return NodeStatus The result of the execution; SUCCESS or FAILURE.
     */
    NodeStatus tick() override;

    private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr);

    //process node
    rclcpp::Node::SharedPtr rosnode;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub;
    bool msgReceived;
    nav_msgs::msg::Odometry odom;
};

/**
 * @brief Transforms a pose from one coordinate frame to another.
 */
class TransformPose : public UWRTSyncActionNode { //TODO: Rename class to whatever your state is named.
    public:
    TransformPose(const std::string& name, const NodeConfiguration& config) //TODO: Rename constructor to match class name.
     : UWRTSyncActionNode(name, config) { }

    /**
     * @brief Declares ports needed by this state.
     * @return PortsList Needed ports.
     */
    static PortsList providedPorts() {
        return {
            InputPort<std::string>("from_frame"),
            InputPort<std::string>("to_frame"),
            InputPort<double>("x"),
            InputPort<double>("y"),
            InputPort<double>("z"),
            InputPort<double>("or"),
            InputPort<double>("op"),
            InputPort<double>("oy"),
            OutputPort<double>("out_x"),
            OutputPort<double>("out_y"),
            OutputPort<double>("out_z"),
            OutputPort<double>("out_or"),
            OutputPort<double>("out_op"),
            OutputPort<double>("out_oy")
        };
    }

    /**
     * @brief Initializes the node.
     * @param node The ROS node belonging to the current process.
     */
    void init(rclcpp::Node::SharedPtr) override;

    /**
     * @brief Executes the node.
     *
     * @return NodeStatus The result of the execution; SUCCESS or FAILURE.
     */
    NodeStatus tick() override;

    private:
    //process node
    rclcpp::Node::SharedPtr rosnode;


};

/**
 * @brief Search State.
 *
 * This state searches for a given frame/object while moving towards it.
 * Its goal is to reduce the covariance of the position estimate from mapping,
 * not to bring the robot all the way to the object. When this state ends, the
 * position of the object should be accurate enough to use ToWorldFrameState and
 * BigMoveState to fully bring the robot to the object.
 */
class Search : public UWRTSyncActionNode {
    public:
    Search(const std::string& name, const NodeConfiguration& config)
     : UWRTSyncActionNode(name, config) { }

    /**
     * @brief Declares ports needed by this state.
     *
     * @return PortsList Needed ports.
     */
    static PortsList providedPorts() {
        return {
            InputPort<std::string>("frame"),
            InputPort<std::string>("target_error"),
            InputPort<std::string>("update_sec")
        };
    }

    /**
     * @brief Initializes the node.
     *
     * @param node The ROS node belonging to the current process.
     */
    void init(rclcpp::Node::SharedPtr) override;

    /**
     * @brief Executes the node.
     * This method will be called once by the tree and can block for as long
     * as it needs for the action to be completed. When execution completes,
     * this method must return either SUCCESS or FAILURE; it CANNOT return
     * IDLE or RUNNING.
     *
     * @return NodeStatus The result of the execution; SUCCESS or FAILURE.
     */
    NodeStatus tick() override;

    private:
    void publishPosition(geometry_msgs::msg::Vector3);
    void publishOrientation(geometry_msgs::msg::Quaternion);

    /**
     * @brief Called when the guess subscription receives a message.
     *
     * @param msg The received message.
     */
    void guessCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr);

    /**
     * @brief Called when the guess subscription receives a message.
     *
     * @param msg The received message.
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr);

    //ROS node
    rclcpp::Node::SharedPtr rosnode;

    //subscriptions
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr guessSub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;

    //publishers
    rclcpp::Publisher<riptide_msgs2::msg::ControllerCommand>::SharedPtr positionPub;
    rclcpp::Publisher<riptide_msgs2::msg::ControllerCommand>::SharedPtr orientationPub;

    //other things
    geometry_msgs::msg::Pose
        guessLocation,
        currentPose;

    double guessError;
    bool
        receivedGuess,
        receivedPos;
};


/**
 * State that actuates stuff on the robot
 */
class Actuate : public UWRTSyncActionNode {
    public:
    Actuate(const std::string& name, const NodeConfiguration& config)
     : UWRTSyncActionNode(name, config) { }

    /**
     * @brief Declares ports needed by this state.
     * @return PortsList Needed ports.
     */
    static PortsList providedPorts() {
        return {
            InputPort<bool>("drop_1"),
            InputPort<bool>("drop_2"),
            InputPort<bool>("clear_dropper_status"),
            InputPort<bool>("arm_torpedo"),
            InputPort<bool>("disarm_torpedo"),
            InputPort<bool>("fire_torpedo_1"),
            InputPort<bool>("fire_torpedo_2"),
            InputPort<bool>("open_claw"),
            InputPort<bool>("close_claw"),
            InputPort<bool>("reset_actuators")
        };
    }

    /**
     * @brief Initializes the node.
     * @param node The ROS node belonging to the current process.
     */
    void init(rclcpp::Node::SharedPtr) override;

    /**
     * @brief Executes the node.
     * This method will be called once by the tree and can block for as long
     * as it needs for the action to be completed. When execution completes,
     * this method must return either SUCCESS or FAILURE; it CANNOT return
     * IDLE or RUNNING.
     *
     * @return NodeStatus The result of the execution; SUCCESS or FAILURE.
     */
    NodeStatus tick() override;

    private:
    //process node
    rclcpp::Node::SharedPtr rosnode;
    rclcpp::Publisher<riptide_msgs2::msg::ActuatorCommand>::SharedPtr publisher;
};

/**
 * @brief Reads out the status of the actuators into behaviortree ports
 *
 */
class GetActuatorStatus : public UWRTSyncActionNode { //TODO: Rename class to whatever your state is named.
    public:
    GetActuatorStatus(const std::string& name, const NodeConfiguration& config) //TODO: Rename constructor to match class name.
     : UWRTSyncActionNode(name, config) { }

    /**
     * @brief Declares ports needed by this state.
     * @return PortsList Needed ports.
     */
    static PortsList providedPorts() {
        return {
            OutputPort<int>("claw_state"),
            OutputPort<int>("torpedo1_state"),
            OutputPort<int>("torpedo2_state"),
            OutputPort<int>("dropper1_state"),
            OutputPort<int>("dropper2_state")
        };
    }

    /**
     * @brief Initializes the node.
     * @param node The ROS node belonging to the current process.
     */
    void init(rclcpp::Node::SharedPtr) override;

    /**
     * @brief Executes the node.
     * This method will be called once by the tree and can block for as long
     * as it needs for the action to be completed. When execution completes,
     * this method must return either SUCCESS or FAILURE; it CANNOT return
     * IDLE or RUNNING.
     *
     * @return NodeStatus The result of the execution; SUCCESS or FAILURE.
     */
    NodeStatus tick() override;

    private:
    void actuatorStateCallback(const riptide_msgs2::msg::ActuatorStatus::SharedPtr);

    //process node
    rclcpp::Node::SharedPtr rosnode;
    rclcpp::Subscription<riptide_msgs2::msg::ActuatorStatus>::SharedPtr statusSub;
    riptide_msgs2::msg::ActuatorStatus latestStatus;
    bool statusReceived;
};

class Wait : public UWRTSyncActionNode { //TODO: Rename class to whatever your state is named.
    public:
    Wait(const std::string& name, const NodeConfiguration& config) //TODO: Rename constructor to match class name.
     : UWRTSyncActionNode(name, config) { }

    /**
     * @brief Declares ports needed by this state.
     * @return PortsList Needed ports.
     */
    static PortsList providedPorts() {
        return {
            InputPort<double>("seconds")
        };
    }

    /**
     * @brief Initializes the node.
     * @param node The ROS node belonging to the current process.
     */
    void init(rclcpp::Node::SharedPtr) override;

    /**
     * @brief Executes the node.
     * This method will be called once by the tree and can block for as long
     * as it needs for the action to be completed. When execution completes,
     * this method must return either SUCCESS or FAILURE; it CANNOT return
     * IDLE or RUNNING.
     *
     * @return NodeStatus The result of the execution; SUCCESS or FAILURE.
     */
    NodeStatus tick() override;

    private:
    //process node
    rclcpp::Node::SharedPtr rosnode;
};

/**
 * @brief BT node that calls and returns the result of the AlignTorpedos action.
 */
class AlignTorpedos : public UWRTSyncActionNode { //TODO: Rename class to whatever your state is named.
    public:
    AlignTorpedos(const std::string& name, const NodeConfiguration& config) //TODO: Rename constructor to match class name.
     : UWRTSyncActionNode(name, config) { }

    /**
     * @brief Declares ports needed by this state.
     * @return PortsList Needed ports.
     */
    static PortsList providedPorts() {
        return {
            InputPort<double>("timeout"),
            InputPort<double>("current_distance"),
            InputPort<double>("goal_distance"),
            OutputPort<double>("x"),
            OutputPort<double>("y"),
            OutputPort<double>("z")
        };
    }

    /**
     * @brief Initializes the node.
     * @param node The ROS node belonging to the current process.
     */
    void init(rclcpp::Node::SharedPtr) override;

    /**
     * @brief Executes the node.
     * This method will be called once by the tree and can block for as long
     * as it needs for the action to be completed. When execution completes,
     * this method must return either SUCCESS or FAILURE; it CANNOT return
     * IDLE or RUNNING.
     *
     * @return NodeStatus The result of the execution; SUCCESS or FAILURE.
     */
    NodeStatus tick() override;

    private:
    //process node
    rclcpp::Node::SharedPtr rosnode;
};

class ActuatorConditions {
    public:
    //register states
    static void registerConditions(BT::BehaviorTreeFactory *factory);
};

/**
 * @brief Reads out the state of the switches into behaviortree ports
 */
class GetSwitchState : public UWRTSyncActionNode {
    public:
    GetSwitchState(const std::string& name, const NodeConfiguration& config) //TODO: Rename constructor to match class name.
     : UWRTSyncActionNode(name, config) { }

    /**
     * @brief Declares ports needed by this state.
     * @return PortsList Needed ports.
     */
    static PortsList providedPorts() {
        return {
            OutputPort<bool>("kill_switch_inserted"),
            OutputPort<bool>("aux_switch_inserted")
        };
    }

    /**
     * @brief Initializes the node.
     * @param node The ROS node belonging to the current process.
     */
    void init(rclcpp::Node::SharedPtr) override;

    /**
     * @brief Executes the node.
     * This method will be called once by the tree and can block for as long
     * as it needs for the action to be completed. When execution completes,
     * this method must return either SUCCESS or FAILURE; it CANNOT return
     * IDLE or RUNNING.
     *
     * @return NodeStatus The result of the execution; SUCCESS or FAILURE.
     */
    NodeStatus tick() override;

    private:
    void robotStateCallback(const riptide_msgs2::msg::RobotState::SharedPtr);

    //process node
    rclcpp::Node::SharedPtr rosnode;
    rclcpp::Subscription<riptide_msgs2::msg::RobotState>::SharedPtr stateSub;
    riptide_msgs2::msg::RobotState latestState;
    bool stateReceived;
};

class ResetOdom : public UWRTSyncActionNode {
    public:
    ResetOdom(const std::string& name, const NodeConfiguration& config) //TODO: Rename constructor to match class name.
     : UWRTSyncActionNode(name, config) {}

    /**
     * @brief Declares ports needed by this state.
     * @return PortsList Needed ports.
     */
    static PortsList providedPorts() {
        return {
            InputPort<double>("x"),
            InputPort<double>("y"),
            InputPort<double>("z"),
            InputPort<double>("or"),
            InputPort<double>("op"),
            InputPort<double>("oy"),
        };
    }

    /**
     * @brief Initializes the node.
     * @param node The ROS node belonging to the current process.
     */
    void init(rclcpp::Node::SharedPtr) override;

    /**
     * @brief Executes the node.
     * This method will be called once by the tree and can block for as long
     * as it needs for the action to be completed. When execution completes,
     * this method must return either SUCCESS or FAILURE; it CANNOT return
     * IDLE or RUNNING.
     *
     * @return NodeStatus The result of the execution; SUCCESS or FAILURE.
     */
    NodeStatus tick() override;

    private:
    rclcpp::Node::SharedPtr rosnode;
    rclcpp::Client<robot_localization::srv::SetPose>::SharedPtr client;
};

#endif // AUTONOMY_H