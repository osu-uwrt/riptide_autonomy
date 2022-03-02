#include "autonomy.h"

using namespace BT;
using namespace std::placeholders;


void FlattenCalculationState::init(rclcpp::Node::SharedPtr node) {
    rosnode = node;
    odomSub = node->create_subscription<nav_msgs::msg::Odometry>(ODOMETRY_TOPIC, 10, std::bind(&FlattenCalculationState::odomCallback, this, _1));

    odomExists = false;
}


NodeStatus FlattenCalculationState::tick() {
    //wait for loc to exist
    RCLCPP_INFO(log, "Waiting for odom...");
    while(!odomExists) {
        rclcpp::spin_some(rosnode);
    }

    RCLCPP_INFO(log, "Odom received.");

    geometry_msgs::msg::Pose goalPose;

    //calculate target position (literally the current pose but 1 meter underwater)
    goalPose.position = currentPose.position;
    goalPose.position.z = getInput<double>("depth").value();

    //calculate target orientation (yaw is the same, pitch and roll are 0)
    tf2::Quaternion currentOrientationQuaternion;
    tf2::fromMsg(currentPose.orientation, currentOrientationQuaternion);

    double roll, pitch, yaw;
    tf2::Matrix3x3(currentOrientationQuaternion).getEulerYPR(yaw, pitch, roll);

    //set roll and pitch to 0, "flattening" the robot
    roll = 0;
    pitch = 0;

    //convert rpy back to quaternion
    tf2::Quaternion newOrientationQuaternion;
    newOrientationQuaternion.setRPY(roll, pitch, yaw);
    newOrientationQuaternion.normalize();
    goalPose.orientation.x = newOrientationQuaternion.getX();
    goalPose.orientation.y = newOrientationQuaternion.getY();
    goalPose.orientation.z = newOrientationQuaternion.getZ();
    goalPose.orientation.w = newOrientationQuaternion.getW();

    //set goal pose
    setOutput<std::string>("x", std::to_string(goalPose.position.x));
    setOutput<std::string>("y", std::to_string(goalPose.position.y));
    setOutput<std::string>("z", std::to_string(goalPose.position.z));
    setOutput<std::string>("orientation_x", std::to_string(goalPose.orientation.x));
    setOutput<std::string>("orientation_y", std::to_string(goalPose.orientation.y));
    setOutput<std::string>("orientation_z", std::to_string(goalPose.orientation.z));
    setOutput<std::string>("orientation_w", std::to_string(goalPose.orientation.w));

    return NodeStatus::SUCCESS;
}


void FlattenCalculationState::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    currentPose = msg->pose.pose;
    odomExists = true;
}