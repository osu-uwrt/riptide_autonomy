#include "autonomy.h"

using namespace BT;


void ShootTorpedoState::init(rclcpp::Node::SharedPtr node) { // TODO: Change BaseState to your new class name
    this->rosnode = node;
}


NodeStatus ShootTorpedoState::tick() { 
    //the actual thing would create a publisher that publishes to whatever the torpedo topic is, but that doesn't exist yet so console print
    RCLCPP_INFO(log, "KABOOM!");
    return NodeStatus::SUCCESS;
}
