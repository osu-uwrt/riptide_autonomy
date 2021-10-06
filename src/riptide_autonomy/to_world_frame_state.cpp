#include "states.h"

/**
 * Source file for the state that converts any relative location to world frame.
 */

using namespace BT;
using namespace states;

PortsList to_world_frame_state::providedPorts() {
    return {
        InputPort<std::string>("object"),
        InputPort<std::string>("relative_x"),
        InputPort<std::string>("relative_y"),
        InputPort<std::string>("relative_z"),
        InputPort<std::string>("relative_orientation_x"),
        InputPort<std::string>("relative_orientation_y"),
        InputPort<std::string>("relative_orientation_z"),
        InputPort<std::string>("relative_orientation_w"),
        OutputPort<std::string>("world_x"),
        OutputPort<std::string>("world_y"),
        OutputPort<std::string>("world_z"),
        OutputPort<std::string>("world_orientation_x"),
        OutputPort<std::string>("world_orientation_y"),
        OutputPort<std::string>("world_orientation_z"),
        OutputPort<std::string>("world_orientation_w")
    };
}


NodeStatus to_world_frame_state::tick() {
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    std::string objectName = getInput<std::string>("object").value();
    geometry_msgs::TransformStamped transform = buffer.lookupTransform(objectName, "world", ros::Time(0), ros::Duration(1.0));

    geometry_msgs::Pose currentPose;
    geometry_msgs::Pose desiredPose;

    currentPose.position.x = std::stod(getInput<std::string>("relative_x").value());
    currentPose.position.y = std::stod(getInput<std::string>("relative_y").value());
    currentPose.position.z = std::stod(getInput<std::string>("relative_z").value());
    currentPose.orientation.x = std::stod(getInput<std::string>("relative_orientation_x").value());
    currentPose.orientation.y = std::stod(getInput<std::string>("relative_orientation_y").value());
    currentPose.orientation.z = std::stod(getInput<std::string>("relative_orientation_z").value());
    currentPose.orientation.w = std::stod(getInput<std::string>("relative_orientation_w").value());

    tf2::doTransform(currentPose, desiredPose, transform);

    setOutput<std::string>("world_x", std::to_string(desiredPose.position.x));
    setOutput<std::string>("world_y", std::to_string(desiredPose.position.y));
    setOutput<std::string>("world_z", std::to_string(desiredPose.position.z));
    setOutput<std::string>("world_orientation_x", std::to_string(desiredPose.orientation.x));
    setOutput<std::string>("world_orientation_y", std::to_string(desiredPose.orientation.y));
    setOutput<std::string>("world_orientation_z", std::to_string(desiredPose.orientation.z));
    setOutput<std::string>("world_orientation_w", std::to_string(desiredPose.orientation.w));

    return NodeStatus::SUCCESS;
}
