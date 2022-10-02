#include "autonomy.h"

using namespace BT;


PortsList ComputeFrameAlignment::providedPorts() {
    return {
        InputPort<double>("x"),
        InputPort<double>("y"),
        InputPort<double>("z"),
        InputPort<std::string>("frameName"),
        OutputPort<double>("out_x"),
        OutputPort<double>("out_y"),
        OutputPort<double>("out_z"),
    };
}


NodeStatus ComputeFrameAlignment::tick() { 
    tf2_ros::Buffer buffer(rosnode->get_clock());
    rclcpp::Time startTime = rosnode->get_clock()->now();

    std::string finalFrame = getInput<std::string>("frameName").value();
    std::string fromOdom = "odom";
    std::string BaseFrame = "tempest/base_link";

    //make a pose  that represents the desired position of the given frame
    geometry_msgs::msg::Pose desiredPose;
    desiredPose.position.x = getInput<double>("x").value();
    desiredPose.position.y = getInput<double>("y").value();
    desiredPose.position.z = getInput<double>("z").value();

    //convert this to base frame
    std::tuple<geometry_msgs::msg::Pose, bool> toBase = transformBetweenFrames(desiredPose, BaseFrame, fromOdom, rosnode);
    if(!(std::get<1>(toBase))){
        return NodeStatus::FAILURE;
    }

    //convert base to odom

    desiredPose = std::get<0>(toBase);

    std::tuple<geometry_msgs::msg::Pose, bool> toFinal = transformBetweenFrames(desiredPose, finalFrame, BaseFrame, rosnode);

    if(!(std::get<1>(toFinal))){
        return NodeStatus::FAILURE;
    }

    geometry_msgs::msg::Pose finalRes = std::get<0>(toFinal);

    //set outputs to the odom coords that get the desired frame to the desired position

    setOutput<double>("out_x", finalRes.position.x);
    setOutput<double>("out_y", finalRes.position.y);
    setOutput<double>("out_z", finalRes.position.z);


    return NodeStatus::SUCCESS;
}
