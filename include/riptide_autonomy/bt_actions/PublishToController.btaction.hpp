#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

class PublishToController : public UWRTActionNode {
    public:
    PublishToController(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config) {
        
    }

    /**
     * @brief Declares ports needed by this node.
     * @return PortsList Needed ports.
     */
    static BT::PortsList providedPorts() {
        return {
            UwrtInput("isOrientation"),
            UwrtInput("mode"),
            UwrtInput("x"),
            UwrtInput("y"),
            UwrtInput("z"),
        };
    }

    /**
     * @brief Initializes ROS peripherals such as publishers, subscribers, actions, services, etc.
     * Anything requiring the ROS node handle to construct should be initialized here. Do not do it in the 
     * constructor or you will be very sad
     */
    void rosInit() override { 
        positionPub = rosnode->create_publisher<riptide_msgs2::msg::ControllerCommand>(CONTROL_LINEAR_TOPIC, 10);
        orientationPub = rosnode->create_publisher<riptide_msgs2::msg::ControllerCommand>(CONTROL_ANGULAR_TOPIC, 10);
    }

    /**
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override {
        //which metric are we publishing?
        bool isOrientation = tryGetOptionalInput<bool>(this, "isOrientation", false);

        //create message to publish
        riptide_msgs2::msg::ControllerCommand cmd;
        cmd.mode = tryGetRequiredInput<int>(this, "mode", 3);
        cmd.setpoint_vect.x = tryGetRequiredInput<double>(this, "x", 0);
        cmd.setpoint_vect.y = tryGetRequiredInput<double>(this, "y", 0);
        cmd.setpoint_vect.z = tryGetRequiredInput<double>(this, "z", 0);
        
        //if in orientation position mode, must fill out quat. vect will not be considered
        if(isOrientation && cmd.mode == riptide_msgs2::msg::ControllerCommand::POSITION) {
            cmd.setpoint_quat = toQuat(cmd.setpoint_vect);
        } 

        //publish to appropriate publisher
        if(isOrientation) {
            orientationPub->publish(cmd);
        } else {
            positionPub->publish(cmd);
        }

        return BT::NodeStatus::SUCCESS;
    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after 
     */
    BT::NodeStatus onRunning() override {
        return BT::NodeStatus::SUCCESS;
    }

    /**
     * @brief Called when the node is halted.
     */
    void onHalted() override {
        
    }

    private:
    rclcpp::Publisher<riptide_msgs2::msg::ControllerCommand>::SharedPtr
        positionPub,
        orientationPub;
};
