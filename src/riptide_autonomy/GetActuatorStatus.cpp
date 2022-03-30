#include "autonomy.h"

using std::placeholders::_1;
using namespace BT;

void GetActuatorStatus::init(rclcpp::Node::SharedPtr node) {
    this->rosnode = node;
}


NodeStatus GetActuatorStatus::tick() { 
    this->statusSub = rosnode->create_subscription<riptide_msgs2::msg::ActuatorStatus> (
        ACTUATOR_STATUS_TOPIC, 
        rclcpp::SensorDataQoS(),
        std::bind(&GetActuatorStatus::actuatorStateCallback, this, _1)
    );

    rclcpp::Time startTime = rosnode->get_clock()->now();

    statusReceived = false;
    while(!statusReceived) {
        rclcpp::spin_some(rosnode);

        if((rosnode->get_clock()->now() - startTime).seconds() > 3) {
            RCLCPP_ERROR(log, "Timed out getting actuator status.");
            return NodeStatus::FAILURE;
        }
    }

    riptide_msgs2::msg::ActuatorStatus status = latestStatus;

    setOutput<int>("claw_state", status.claw_state);
    setOutput<int>("torpedo1_state", status.torpedo1_state);
    setOutput<int>("torpedo2_state", status.torpedo2_state);
    setOutput<int>("dropper1_state", status.dropper1_state);
    setOutput<int>("dropper2_state", status.dropper2_state);

    return NodeStatus::SUCCESS;
}

void GetActuatorStatus::actuatorStateCallback(const riptide_msgs2::msg::ActuatorStatus::SharedPtr msg) {
    latestStatus = *msg;
    statusReceived = true;
}
