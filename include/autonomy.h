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

using namespace BT;

//define logger for RCLCPP_INFO, RCLCPP_WARN, and RCLCPP_ERROR
#define log rclcpp::get_logger("rclcpp")

//useful constant values for autonomy
const std::string
    STEADY_TOPIC = "steady",
    ODOMETRY_TOPIC = "odometry/filtered",
    POSITION_TOPIC = "position",
    ORIENTATION_TOPIC = "orientation",
    ANGULAR_VELOCITY_TOPIC = "angular_velocity",
    LINEAR_VELOCITY_TOPIC = "linear_velocity";

geometry_msgs::msg::Pose doTransform(geometry_msgs::msg::Pose, geometry_msgs::msg::TransformStamped);

/**
 * @brief A BT SyncActionNode with an init() method that takes a ROS node as a parameter.
 * This class should be inherited by every state used by the robot to avoid creating a new 
 * ros node every time a state change happens.
 */
class UWRTSyncActionNode : public BT::SyncActionNode {
    public:
    UWRTSyncActionNode(const std::string& name, const BT::NodeConfiguration& config)
     : BT::SyncActionNode(name, config) { };

    static PortsList providedPorts();
    NodeStatus tick() override;
    virtual void init(rclcpp::Node::SharedPtr node) = 0;
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
 * @brief Move state.
 * 
 * This state moves the robot to a specified position and/or orientation. Optionally,
 * this state can set an angular velocity.
 */
class BigMoveState : public UWRTSyncActionNode {
    public: 
    BigMoveState(const std::string& name, const NodeConfiguration& config)
     : UWRTSyncActionNode(name, config) { }
    
    /**
     * @brief Initializes the state.
     * 
     * @param node The ROS node belonging to the current process
     */
    void init(rclcpp::Node::SharedPtr) override;

    /**
     * @brief Ticks the action.
     * All exeuction of the state goes in this method. It will
     * only be called once and must return either SUCCESS OR FAILURE
     * 
     * @return NodeStatus The status of the node after the execution completes
     */
    NodeStatus tick() override;

    /**
     * @brief Declares the ports needed by the state.
     * 
     * @return PortsList Needed ports
     */
    static PortsList providedPorts() {
        return {
            InputPort<std::string>("x"),
            InputPort<std::string>("y"),
            InputPort<std::string>("z"),
            InputPort<std::string>("orientation_x"),
            InputPort<std::string>("orientation_y"),
            InputPort<std::string>("orientation_z"),
            InputPort<std::string>("orientation_w"),
            InputPort<std::string>("v_roll"),
            InputPort<std::string>("v_pitch"),
            InputPort<std::string>("v_yaw")
        };
    }

    private:
    /**
     * @brief Publishes the goal position, orientation, and/or angular velocity
     */
    void publishGoalInformation();
    
    /**
     * @brief Converts a quaternion to euler angles
     */
    geometry_msgs::msg::Vector3 toRPY(geometry_msgs::msg::Quaternion);
    
    /**
     * @brief Called when odometry subscription received a message.
     * 
     * @param msg The message received.
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr);
    
    /**
     * @brief Called before the main loop enters.
     * This method reads the behaviortree ports and determines what the 
     * state's goal is (will it set position? Orientation? Both?). 
     */
    void resolveGoalPosition();

    static constexpr double THRESHOLD = 0.1;

    //process' ros node
    rclcpp::Node::SharedPtr rosnode;

    bool
        hasEntered,
        odomExists,
        publishPosition,
        publishOrientation,
        publishAngularVelocity;
    
    //pose data
    geometry_msgs::msg::Pose
        currentPose,
        goalPose;

    geometry_msgs::msg::Vector3 
        goalAngVelocity;

    //ROS subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscriber;

    //ROS publishers
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr    positionPub;
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr orientationPub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr    angVelocityPub;
};

/**
 * @brief Flatten state.
 * 
 * This state calculates the position and orientation that the robot should move to in 
 * order to properly flatten itself.
 */
class FlattenCalculationState : public UWRTSyncActionNode {
    public:
    FlattenCalculationState(const std::string& name, const NodeConfiguration& config)
     : UWRTSyncActionNode(name, config) { }

    /**
     * @brief Declares ports needed by this state.
     * 
     * @return PortsList Needed ports.
     */
    static PortsList providedPorts() {
        return {
            InputPort<double>("depth"),
            OutputPort<std::string>("x"),
            OutputPort<std::string>("y"),
            OutputPort<std::string>("z"),
            OutputPort<std::string>("orientation_x"),
            OutputPort<std::string>("orientation_y"),
            OutputPort<std::string>("orientation_z"),
            OutputPort<std::string>("orientation_w")
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
    /**
     * @brief Called when the odometry subscription receives a message.
     * 
     * @param msg The received message.
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr);

    //process node
    rclcpp::Node::SharedPtr rosnode;

    //odometry stuff
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;
    bool odomExists;
    geometry_msgs::msg::Pose currentPose;
};

/**
 * @brief Line-drive state
 * 
 * This state calculates the position that the robot should move to in order 
 * to drive directly through a target frame.
 */
class LineDriveCalcState : public UWRTSyncActionNode {
    public:
    LineDriveCalcState(const std::string& name, const NodeConfiguration& config)
     : UWRTSyncActionNode(name, config) { }

    /**
     * @brief Declares ports needed by this state.
     * 
     * @return PortsList Needed ports.
     */
    static PortsList providedPorts() {
        return {
            InputPort("frame"),
            InputPort("extra_distance"),
            OutputPort("x_out"),
            OutputPort("y_out"),
            OutputPort("z_out")
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
    /**
     * @brief Called when the odometry subscription receives a message.
     * 
     * @param msg The received message.
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr);

    rclcpp::Node::SharedPtr rosnode;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;
    geometry_msgs::msg::Pose currentPose;

    bool odomExists;
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
class SearchState : public UWRTSyncActionNode {
    public:
    SearchState(const std::string& name, const NodeConfiguration& config)
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

    /**
     * @brief Calculates the distance between two given points.
     * 
     * @param point1 The first point
     * @param point2 The second point
     * @return double The distance between point1 and point2.
     */
    double distance(geometry_msgs::msg::Point , geometry_msgs::msg::Point );

    //ROS node
    rclcpp::Node::SharedPtr rosnode;

    //subscriptions
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr guessSub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;

    //publishers
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr positionPub;
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr orientationPub;

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
 * @brief State that converts relative coordinates to global (world frame) ones.
 * 
 * This state receives a frame/object name, a set of coordinates relative to that 
 * frame/object, and an orientation relative to that frame/object, and transforms 
 * those coordinates to global or "world frame" coordinates. This state can be used,
 * for example, if you want to move the robot 1 meter directly in front of the gate.
 * Use gate_frame as the frame name, the coordinates 1, 0, 0, and the orientation 
 * 0, 0, -1, 1, and then use the move state with the output values from this state.
 */
class ToWorldFrameState : public UWRTSyncActionNode {
    public:
    ToWorldFrameState(const std::string& name, const NodeConfiguration& config)
     : UWRTSyncActionNode(name, config) { }

    /**
     * @brief Declares ports needed by this state.
     * 
     * @return PortsList Needed ports.
     */
    static PortsList providedPorts() {
        return {
            InputPort<std::string>("object"),
            InputPort<std::string>("relative_x"),
            InputPort<std::string>("relative_y"),
            InputPort<std::string>("relative_z"),
            InputPort<std::string>("relative_orientation_x"),
            InputPort<std::string>("relative_orientation_y"),
            InputPort<std::string>("relative_orientation_z"),
            InputPort<std::string>("relative_orientation_w"),
            OutputPort<std::string>("world_x"),
            OutputPort<std::string>("world_y"),
            OutputPort<std::string>("world_z"),
            OutputPort<std::string>("world_orientation_x"),
            OutputPort<std::string>("world_orientation_y"),
            OutputPort<std::string>("world_orientation_z"),
            OutputPort<std::string>("world_orientation_w")
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
    rclcpp::Node::SharedPtr rosnode;
};

/**
 * @brief Velocity state.
 * 
 * This state drives the robot at a constant velocity while fixed at its
 * current orientation (its orientation when the state starts)
 */
class VelocityState : public UWRTSyncActionNode {
    public:
    VelocityState(const std::string& name, const NodeConfiguration& config)
     : UWRTSyncActionNode(name, config) { }

    /**
     * @brief Declares ports needed by this state.
     * 
     * @return PortsList Needed ports.
     */
    static PortsList providedPorts() {
        return {
            InputPort<std::string>("time"),
            InputPort<std::string>("x_velocity"),
            InputPort<std::string>("y_velocity"),
            InputPort<std::string>("z_velocity")
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
    /**
     * @brief Called when the odometry subscription receives a message.
     * 
     * @param msg The received message.
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr);

    rclcpp::Node::SharedPtr rosnode;

    //publishers 
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr velocityPub;
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr orientationPub;

    //subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;

    //goal information
    geometry_msgs::msg::Vector3 velocities;
    geometry_msgs::msg::Quaternion orientation;

    //other information
    geometry_msgs::msg::Pose currentPose;

    bool
        hasEntered,
        odomDataExists;

    int startTime;
};

#endif // AUTONOMY_H