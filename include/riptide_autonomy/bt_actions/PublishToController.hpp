#pragma once

#include "riptide_autonomy/autonomy_base.hpp"
#include "riptide_autonomy/uwrt_node_types.hpp"

#include <riptide_msgs2/msg/controller_command.hpp>

class PublishToController : public UWRTActionNode {
    public:
    PublishToController(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config) {
        
    }

    /**
     * @brief Declares ports needed by this node.
     * @return PortsList Needed ports.
     */
    static UwrtPortInformation portInformation() {
        return {
            UwrtInput("is_orientation", PORT_REQUIRED,
                "1 if publishing RPY orientation, 0 otherwise"),

            UwrtInput("mode", PORT_REQUIRED,
                "mode integer, corresponding to any mode in the ControllerCommand message."),

            UwrtInput("x", PORT_REQUIRED,
                "If is_orientation == 1, this is roll. Otherwise, this is X."),

            UwrtInput("y", PORT_REQUIRED,
                "If is_orientation == 1, this is pitch. Otherwise, this is Y."),
                
            UwrtInput("z", PORT_REQUIRED,
                "If is_orientation == 1, this is yaw. Otherwise, this is Z."),
        };
    }

    /**
     * @brief Initializes ROS peripherals such as publishers, subscribers, actions, services, etc.
     * Anything requiring the ROS node handle to construct should be initialized here. Do not do it in the 
     * constructor or you will be very sad
     */
    void rosInit() override { 
        positionPub = rosNode()->create_publisher<riptide_msgs2::msg::ControllerCommand>(CONTROL_LINEAR_TOPIC, 10);
        orientationPub = rosNode()->create_publisher<riptide_msgs2::msg::ControllerCommand>(CONTROL_ANGULAR_TOPIC, 10);
    }

    /**
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override {
        //which metric are we publishing?
        bool isOrientation = tryGetOptionalInput<bool>("isOrientation", false);

        //create message to publish
        riptide_msgs2::msg::ControllerCommand cmd;
        cmd.mode = tryGetRequiredInput<int>("mode", 3);
        cmd.setpoint_vect.x = tryGetRequiredInput<double>("x", 0);
        cmd.setpoint_vect.y = tryGetRequiredInput<double>("y", 0);
        cmd.setpoint_vect.z = tryGetRequiredInput<double>("z", 0);
        
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
