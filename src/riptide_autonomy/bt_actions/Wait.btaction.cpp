#include "bt_actions/Wait.h"

using namespace BT;


PortsList Wait::providedPorts() {
    return {
        BT::InputPort<double>("seconds")
    };
}


NodeStatus Wait::tick() {
    double seconds = getInput<double>("seconds").value();

    rclcpp::Time start = rosnode->get_clock()->now();
    while((rosnode->get_clock()->now() - start).seconds() < seconds) {
        rclcpp::spin_some(rosnode);
    }

    return NodeStatus::SUCCESS;
}
