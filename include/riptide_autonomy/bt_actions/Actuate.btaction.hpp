#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

class Actuate : public UWRTActionNode {
    public:
    Actuate(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config) { }

    /**
     * @brief Declares ports needed by this node.
     * @return PortsList Needed ports.
     */
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<bool>("drop_1"),
            BT::InputPort<bool>("drop_2"),
            BT::InputPort<bool>("clear_dropper_status"),
            BT::InputPort<bool>("arm_torpedo"),
            BT::InputPort<bool>("disarm_torpedo"),
            BT::InputPort<bool>("fire_torpedo_1"),
            BT::InputPort<bool>("fire_torpedo_2"),
            BT::InputPort<bool>("open_claw"),
            BT::InputPort<bool>("close_claw"),
            BT::InputPort<bool>("reset_actuators")
        };
    }

    /**
     * @brief Initializes ROS peripherals such as publishers, subscribers, actions, services, etc.
     * Anything requiring the ROS node handle to construct should be initialized here. Do not do it in the 
     * constructor or you will be very sad
     */
    void rosInit() override {
        publisher = rosnode->create_publisher<riptide_msgs2::msg::ActuatorCommand>(ACTUATOR_COMMAND_TOPIC, 10);
    }

    /**
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override {
        RCLCPP_WARN(log, "The Actuate node is deprecated. Use the fancy action client actuators!");

        //get inputs from BT
        bool 
            drop1                = tryGetOptionalInput<bool>(this, "drop_1", false),
            drop2                = tryGetOptionalInput<bool>(this, "drop_2", false),
            clear_dropper_status = tryGetOptionalInput<bool>(this, "clear_dropper_status", false),
            arm_torpedo          = tryGetOptionalInput<bool>(this, "arm_torpedo", false),
            disarm_torpedo       = tryGetOptionalInput<bool>(this, "disarm_torpedo", false),
            fire_torpedo_1       = tryGetOptionalInput<bool>(this, "fire_torpedo_1", false),
            fire_torpedo_2       = tryGetOptionalInput<bool>(this, "fire_torpedo_2", false),
            open_claw            = tryGetOptionalInput<bool>(this, "open_claw", false),
            close_claw           = tryGetOptionalInput<bool>(this, "close_claw", false),
            reset_actuators      = tryGetOptionalInput<bool>(this, "reset_actuators", false);

        //set wanted actuator states
        riptide_msgs2::msg::ActuatorCommand cmd;
        cmd.drop_2 = drop2;
        cmd.drop_1 = drop1;
        cmd.clear_dropper_status = clear_dropper_status;
        cmd.arm_torpedo = arm_torpedo;
        cmd.disarm_torpedo = disarm_torpedo;
        cmd.fire_torpedo_1 = fire_torpedo_1;
        cmd.fire_torpedo_2 = fire_torpedo_2;
        cmd.open_claw = open_claw;
        cmd.close_claw = close_claw;
        cmd.reset_actuators = reset_actuators;

        publisher->publish(cmd);

        return BT::NodeStatus::SUCCESS;
    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after 
     */
    BT::NodeStatus onRunning() override {
        return BT::NodeStatus::SUCCESS; //node should complete when called
    }

    /**
     * @brief Called when the node is halted.
     */
    void onHalted() override { }

    private:
    rclcpp::Publisher<riptide_msgs2::msg::ActuatorCommand>::SharedPtr publisher;
};
