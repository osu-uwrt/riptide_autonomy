#include "bt_actions/TransformPose.h"

using namespace BT;


PortsList TransformPose::providedPorts() {
    return {
        BT::InputPort<std::string>("from_frame"),
        BT::InputPort<std::string>("to_frame"),
        BT::InputPort<double>("x"),
        BT::InputPort<double>("y"),
        BT::InputPort<double>("z"),
        BT::InputPort<double>("or"),
        BT::InputPort<double>("op"),
        BT::InputPort<double>("oy"),
        BT::OutputPort<double>("out_x"),
        BT::OutputPort<double>("out_y"),
        BT::OutputPort<double>("out_z"),
        BT::OutputPort<double>("out_or"),
        BT::OutputPort<double>("out_op"),
        BT::OutputPort<double>("out_oy")
    };
}


NodeStatus TransformPose::tick() {
    //tf buffer and listener for listening to data
    tf2_ros::Buffer buffer(rosnode->get_clock());
    tf2_ros::TransformListener listener(buffer);

    // get frame names
    std::string
        fromFrame = getInput<std::string>("from_frame").value(),
        toFrame = getInput<std::string>("to_frame").value();

    geometry_msgs::msg::Pose original;
    original.position.x = getInput<double>("x").value();
    original.position.y = getInput<double>("y").value();
    original.position.z = getInput<double>("z").value();
    
    //convert original rpy to quaternion and set that
    geometry_msgs::msg::Vector3 originalRPY;
    originalRPY.x = getInput<double>("or").value();
    originalRPY.y = getInput<double>("op").value();
    originalRPY.z = getInput<double>("oy").value();
    original.orientation = toQuat(originalRPY);
    

    std::tuple<geometry_msgs::msg::Pose, bool> res = transformBetweenFrames(original, toFrame, fromFrame, rosnode);
    geometry_msgs::msg::Pose transformed = std::get<0>(res);

    geometry_msgs::msg::Vector3 transformedRPY = toRPY(transformed.orientation);

    if(std::get<1>(res)){
    //RCLCPP_INFO(log, "transformed orientation: %f, %f, %f, %f", transformed.orientation.x, transformed.orientation.y, transformed.orientation.z, transformed.orientation.w);

    //set output ports
    setOutput<double>("out_x", transformed.position.x);
    setOutput<double>("out_y", transformed.position.y);
    setOutput<double>("out_z", transformed.position.z);

    //convert orientation back to RPY and return that
    RCLCPP_INFO(log, "transformed rpy: %f, %f ,%f", transformedRPY.x, transformedRPY.y, transformedRPY.z);
    setOutput<double>("out_or", transformedRPY.x);
    setOutput<double>("out_op", transformedRPY.y);
    setOutput<double>("out_oy", transformedRPY.z);
    return NodeStatus::SUCCESS;
    } else{
        RCLCPP_ERROR(log, "Failed to look up transform from %s to %s!", fromFrame.c_str(), toFrame.c_str());
        return NodeStatus::FAILURE;
    }
}
