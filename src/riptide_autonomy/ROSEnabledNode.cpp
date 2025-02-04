#include "riptide_autonomy/uwrt_node_types.hpp"

std::shared_ptr<tf2_ros::Buffer> ROSEnabledNode::tfBuffer = nullptr;
std::shared_ptr<tf2_ros::TransformListener> ROSEnabledNode::tfListener = nullptr;

void ROSEnabledNode::staticInit(rclcpp::Node::SharedPtr node) {
    if(!tfBuffer)
    {
        tfBuffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    }

    if(!tfListener)
    {
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
    }
}

void ROSEnabledNode::staticDeinit() {
    if(tfBuffer)
    {
        tfBuffer.reset();
    }

    if(tfListener)
    {
        tfListener.reset();
    }
}

void ROSEnabledNode::init(rclcpp::Node::SharedPtr node) {
    this->rosnode = node;
    rosInit();
}

const rclcpp::Node::SharedPtr ROSEnabledNode::rosNode() const {
    return rosnode;
}
