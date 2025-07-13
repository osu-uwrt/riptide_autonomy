#include "riptide_autonomy/UwrtBtNode.hpp"

std::shared_ptr<tf2_ros::Buffer> UwrtBtNode::tfBuffer = nullptr;
std::shared_ptr<tf2_ros::TransformListener> UwrtBtNode::tfListener = nullptr;

void UwrtBtNode::staticInit(rclcpp::Node::SharedPtr node) {
    if(!tfBuffer)
    {
        tfBuffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    }

    if(!tfListener)
    {
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
    }
}


void UwrtBtNode::staticDeinit() {
    if(tfBuffer)
    {
        tfBuffer.reset();
    }

    if(tfListener)
    {
        tfListener.reset();
    }
}


void UwrtBtNode::init(rclcpp::Node::SharedPtr node) {
    this->rosnode = node;
    rosInit();
}


bool UwrtBtNode::lookupTransform(
    const std::string& fromFrame,
    const std::string& toFrame,
    geometry_msgs::msg::TransformStamped& transform,
    bool useCurrentTime)
{
    try {
        tf2::TimePoint tp = (useCurrentTime ? tf2_ros::fromRclcpp(this->rosnode->get_clock()->now()) : tf2::TimePointZero);
        transform = tfBuffer->lookupTransform(toFrame, fromFrame, tp);
        return true;
    } catch(tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(rosnode->get_logger(), *rosnode->get_clock(), 1000, "Failed to look up transform from %s to %s (%s)", fromFrame.c_str(), toFrame.c_str(), ex.what());
    }
    
    return false;
}
