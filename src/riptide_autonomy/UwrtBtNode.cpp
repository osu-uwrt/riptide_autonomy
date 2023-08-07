#include "riptide_autonomy/UwrtBtNode.hpp"

std::shared_ptr<tf2_ros::Buffer> UwrtBtNode::tfBuffer = nullptr;
std::shared_ptr<tf2_ros::TransformListener> UwrtBtNode::tfListener = nullptr;

#define INIT_IF_NEEDED(variable, val) \
    do { \
        if(!variable) { \
            variable = val; \
        } \
    } while (0)

#define DEINIT_IF_NEEDED(variable) \
    do { \
        if(variable) { \
            variable.reset(); \
        } \
    } while (0)

void UwrtBtNode::staticInit(rclcpp::Node::SharedPtr node) {
    INIT_IF_NEEDED(tfBuffer, std::make_shared<tf2_ros::Buffer>(node->get_clock()));
    INIT_IF_NEEDED(tfListener, std::make_shared<tf2_ros::TransformListener>(*tfBuffer));
}


void UwrtBtNode::staticDeinit() {
    DEINIT_IF_NEEDED(tfBuffer);
    DEINIT_IF_NEEDED(tfListener);
}


void UwrtBtNode::init(rclcpp::Node::SharedPtr node) {
    this->rosnode = node;
    rosInit();
}
