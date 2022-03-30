#include "autonomy.h"

using namespace BT;

void WaitState::init(rclcpp::Node::SharedPtr node) { // TODO: Change BaseState to your new class name
    this->rosnode = node;
}


NodeStatus WaitState::tick() { //TODO: Change BaseState to your new class name  
    double seconds = getInput<double>("seconds").value();

    rclcpp::Time start = rosnode->get_clock()->now();
    while((rosnode->get_clock()->now() - start).seconds() < seconds) {
        rclcpp::spin_some(rosnode);
    }

    return NodeStatus::SUCCESS;
}
