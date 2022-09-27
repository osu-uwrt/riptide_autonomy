#include "bt_actions/PublishToController.h"

using namespace BT;


PortsList PublishToController::providedPorts() {
    return {
        BT::InputPort<bool>("isOrientation"),
        BT::InputPort<int>("mode"),
        BT::InputPort<double>("x"),
        BT::InputPort<double>("y"),
        BT::InputPort<double>("z"),
    };
}


NodeStatus PublishToController::tick() {
    auto positionPub = rosnode->create_publisher<riptide_msgs2::msg::ControllerCommand>(POSITION_TOPIC, 10);
    auto orientationPub = rosnode->create_publisher<riptide_msgs2::msg::ControllerCommand>(ORIENTATION_TOPIC, 10);

    //which metric are we publishing?
    bool isOrientation = getInput<bool>("isOrientation").value();

    //create message to publish
    riptide_msgs2::msg::ControllerCommand cmd;
    cmd.mode = getInput<int>("mode").value();
    cmd.setpoint_vect.x = getInput<double>("x").value();
    cmd.setpoint_vect.y = getInput<double>("y").value();
    cmd.setpoint_vect.z = getInput<double>("z").value();
    
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

    return NodeStatus::SUCCESS;
}
