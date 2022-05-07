#include "autonomy.h"

using namespace BT;

void Actuate::init(rclcpp::Node::SharedPtr node) {
    this->rosnode = node;
    
    this->publisher = rosnode->create_publisher<riptide_msgs2::msg::ActuatorCommand>(ACTUATOR_COMMAND_TOPIC, 10);
}


NodeStatus Actuate::tick() {
    //get inputs from BT
    Optional<bool> 
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

    return NodeStatus::SUCCESS;
}
