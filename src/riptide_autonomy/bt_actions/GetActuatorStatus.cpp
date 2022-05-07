#include "autonomy.h"

using std::placeholders::_1;
using namespace BT;

void GetActuatorStatus::init(rclcpp::Node::SharedPtr node) {
    this->rosnode = node;

    this->statusSub = rosnode->create_subscription<riptide_msgs2::msg::ActuatorStatus> (
        ACTUATOR_STATUS_TOPIC, 
        rclcpp::SensorDataQoS(),
        std::bind(&GetActuatorStatus::actuatorStateCallback, this, _1)
    );
}


NodeStatus GetActuatorStatus::tick() { 
    rclcpp::Time startTime = rosnode->get_clock()->now();
    statusReceived = false;

    while(!statusReceived) {
        rclcpp::spin_some(rosnode);

        if((rosnode->get_clock()->now() - startTime).seconds() > 3) {
            RCLCPP_ERROR(log, "Timed out getting actuator status.");
            return NodeStatus::FAILURE;
        }
    }

    setOutput<int>("claw_state", latestStatus.claw_state);
    setOutput<int>("torpedo1_state", latestStatus.torpedo1_state);
    setOutput<int>("torpedo2_state", latestStatus.torpedo2_state);
    setOutput<int>("dropper1_state", latestStatus.dropper1_state);
    setOutput<int>("dropper2_state", latestStatus.dropper2_state);

    return NodeStatus::SUCCESS;
}

void GetActuatorStatus::actuatorStateCallback(const riptide_msgs2::msg::ActuatorStatus::SharedPtr msg) {
    latestStatus = *msg;
    statusReceived = true;
}
