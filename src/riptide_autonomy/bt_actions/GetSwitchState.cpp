#include "autonomy.h"

using std::placeholders::_1;
using namespace BT;

void GetSwitchState::init(rclcpp::Node::SharedPtr node) {
    this->rosnode = node;

    this->stateSub = rosnode->create_subscription<riptide_msgs2::msg::RobotState> (
        ROBOT_STATE_TOPIC,
        rclcpp::SensorDataQoS(),
        std::bind(&GetSwitchState::robotStateCallback, this, _1)
    );
}


NodeStatus GetSwitchState::tick() {
    rclcpp::Time startTime = rosnode->get_clock()->now();
    stateReceived = false;

    while(!stateReceived) {
        rclcpp::spin_some(rosnode);

        if((rosnode->get_clock()->now() - startTime).seconds() > 3) {
            RCLCPP_ERROR(log, "Timed out getting switch state.");
            return NodeStatus::FAILURE;
        }
    }

    setOutput<bool>("kill_switch_inserted", latestState.kill_switch_inserted);
    setOutput<bool>("aux_switch_inserted", latestState.aux_switch_inserted);

    return NodeStatus::SUCCESS;
}

void GetSwitchState::robotStateCallback(const riptide_msgs2::msg::RobotState::SharedPtr msg) {
    latestState = *msg;
    stateReceived = true;
}
