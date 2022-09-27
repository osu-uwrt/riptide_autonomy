#include "bt_actions/GetActuatorStatus.h"

using namespace BT;
using std::placeholders::_1;


bool statusReceived = false;
riptide_msgs2::msg::ActuatorStatus latestStatus;


static void statusCallback(const riptide_msgs2::msg::ActuatorStatus::SharedPtr msg) {
    latestStatus = *msg;
    statusReceived = true;
}


PortsList GetActuatorStatus::providedPorts() {
    return {
        BT::OutputPort<int>("claw_state"),
        BT::OutputPort<int>("torpedo1_state"),
        BT::OutputPort<int>("torpedo2_state"),
        BT::OutputPort<int>("dropper1_state"),
        BT::OutputPort<int>("dropper2_state")
    };
}


NodeStatus GetActuatorStatus::tick() { 
    statusReceived = false; //...so that we force the node to collect another reading

    auto statusSub = rosnode->create_subscription<riptide_msgs2::msg::ActuatorStatus> (
        ACTUATOR_STATUS_TOPIC, 
        rclcpp::SensorDataQoS(),
        std::bind(&statusCallback, _1)
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

    setOutput<int>("claw_state", latestStatus.claw_state);
    setOutput<int>("torpedo1_state", latestStatus.torpedo1_state);
    setOutput<int>("torpedo2_state", latestStatus.torpedo2_state);
    setOutput<int>("dropper1_state", latestStatus.dropper1_state);
    setOutput<int>("dropper2_state", latestStatus.dropper2_state);

    return NodeStatus::SUCCESS;
}
