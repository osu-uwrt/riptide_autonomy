#include "bt_actions/GetSwitchState.h"

using std::placeholders::_1;
using namespace BT;


bool stateReceived = false;
riptide_msgs2::msg::RobotState latestState;


PortsList GetSwitchState::providedPorts() {
    return {
        BT::OutputPort<bool>("kill_switch_inserted"),
        BT::OutputPort<bool>("aux_switch_inserted")
    };
}


static void robotStateCallback(const riptide_msgs2::msg::RobotState::SharedPtr msg) {
    latestState = *msg;
    stateReceived = true;
}


NodeStatus GetSwitchState::tick() {
    stateReceived = false; //force node to collect another message

    auto stateSub = rosnode->create_subscription<riptide_msgs2::msg::RobotState> (
        ROBOT_STATE_TOPIC,
        rclcpp::SensorDataQoS(),
        std::bind(&robotStateCallback, _1)
    );

    rclcpp::Time startTime = rosnode->get_clock()->now();
    stateReceived = false;

    while(!stateReceived) {
        // rclcpp::spin_some(rosnode);

        if((rosnode->get_clock()->now() - startTime).seconds() > 3) {
            RCLCPP_ERROR(log, "Timed out getting switch state.");
            return NodeStatus::FAILURE;
        }
    }

    setOutput<bool>("kill_switch_inserted", latestState.kill_switch_inserted);
    setOutput<bool>("aux_switch_inserted", latestState.aux_switch_inserted);

    return NodeStatus::SUCCESS;
}
