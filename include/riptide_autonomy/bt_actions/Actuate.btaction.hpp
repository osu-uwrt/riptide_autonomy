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
        //get inputs from BT
        BT::Optional<bool> 
            drop1                = getInput<bool>("drop_1"),
            drop2                = getInput<bool>("drop_2"),
            clear_dropper_status = getInput<bool>("clear_dropper_status"),
            arm_torpedo          = getInput<bool>("arm_torpedo"),
            disarm_torpedo       = getInput<bool>("disarm_torpedo"),
            fire_torpedo_1       = getInput<bool>("fire_torpedo_1"),
            fire_torpedo_2       = getInput<bool>("fire_torpedo_2"),
            open_claw            = getInput<bool>("open_claw"),
            close_claw           = getInput<bool>("close_claw"),
            reset_actuators      = getInput<bool>("reset_actuators");

        //set wanted actuator states
        riptide_msgs2::msg::ActuatorCommand cmd;
        
        if(drop1.has_value()) {
            cmd.drop_1 = drop1.value();
        }

        if(drop2.has_value()) {
            cmd.drop_2 = drop2.value();
        }

        if(clear_dropper_status.has_value()) {
            cmd.clear_dropper_status = clear_dropper_status.value();
        }

        if(arm_torpedo.has_value()) {
            cmd.arm_torpedo = arm_torpedo.value();
        }

        if(disarm_torpedo.has_value()) {
            cmd.disarm_torpedo = disarm_torpedo.value();
        }

        if(fire_torpedo_1.has_value()) {
            cmd.fire_torpedo_1 = fire_torpedo_1.value();
        }

        if(fire_torpedo_2.has_value()) {
            cmd.fire_torpedo_2 = fire_torpedo_2.value();
        }

        if(open_claw.has_value()) {
            cmd.open_claw = open_claw.value();
        }

        if(close_claw.has_value()) {
            cmd.close_claw = close_claw.value();
        }

        if(reset_actuators.has_value()) {
            cmd.reset_actuators = reset_actuators.value();
        }

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
