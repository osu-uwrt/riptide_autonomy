#include "autonomy.h"

using namespace BT;


void ToWorldFrameState::init(rclcpp::Node::SharedPtr node) {
    rosnode = node;
}


NodeStatus ToWorldFrameState::tick() {
    //start buffer client to look up transform
    RCLCPP_INFO(log, "Starting TF2 Buffer Server");
    tf2_ros::BufferClient buffer(rosnode, "tf2_buffer_server");
    buffer.waitForServer();

    RCLCPP_INFO(log, "TF2 Buffer Server connected.");

    //look up object (should be broadcasted from mapping)
    std::string objectName = getInput<std::string>("object").value();
    geometry_msgs::msg::TransformStamped transform = buffer.lookupTransform("world", objectName, tf2::TimePointZero, tf2::durationFromSec(1.0));

    geometry_msgs::msg::Pose currentPose;
    geometry_msgs::msg::Pose desiredPose;
    
    //get current pose to translate
    currentPose.position.x = std::stod(getInput<std::string>("relative_x").value());
    currentPose.position.y = std::stod(getInput<std::string>("relative_y").value());
    currentPose.position.z = std::stod(getInput<std::string>("relative_z").value());
    currentPose.orientation.x = std::stod(getInput<std::string>("relative_orientation_x").value());
    currentPose.orientation.y = std::stod(getInput<std::string>("relative_orientation_y").value());
    currentPose.orientation.z = std::stod(getInput<std::string>("relative_orientation_z").value());
    currentPose.orientation.w = std::stod(getInput<std::string>("relative_orientation_w").value());

    //transform the current pose into desired
    tf2::doTransform(currentPose, desiredPose, transform);

    //output desired
    setOutput<std::string>("world_x", std::to_string(desiredPose.position.x));
    setOutput<std::string>("world_y", std::to_string(desiredPose.position.y));
    setOutput<std::string>("world_z", std::to_string(desiredPose.position.z));
    setOutput<std::string>("world_orientation_x", std::to_string(desiredPose.orientation.x));
    setOutput<std::string>("world_orientation_y", std::to_string(desiredPose.orientation.y));
    setOutput<std::string>("world_orientation_z", std::to_string(desiredPose.orientation.z));
    setOutput<std::string>("world_orientation_w", std::to_string(desiredPose.orientation.w));

    RCLCPP_INFO(log, "World Frame Calculated.");

    return NodeStatus::SUCCESS;
}

