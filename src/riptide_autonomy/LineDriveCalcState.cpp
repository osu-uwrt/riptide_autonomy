#include "autonomy.h"

using namespace BT;
using namespace std::placeholders;
    
    
void LineDriveCalcState::init(rclcpp::Node::SharedPtr node) {
    rosnode = node;
    odomSub = node->create_subscription<nav_msgs::msg::Odometry>(ODOMETRY_TOPIC, 10, std::bind(&LineDriveCalcState::odomCallback, this, _1));
}


NodeStatus LineDriveCalcState::tick() {
    //wait for localization data
    RCLCPP_INFO(log, "Waiting for loc data");

    while(!odomExists) {
        rclcpp::spin_some(rosnode);
    }

    //get frame from TF
    RCLCPP_INFO(log, "Starting TF2 Buffer Server");
    tf2_ros::BufferClient buffer(rosnode, "tf2_buffer_server");
    buffer.waitForServer();

    RCLCPP_INFO(log, "TF2 Buffer Server connected.");

    //look up object (should be broadcasted from mapping)
    std::string objectName = getInput<std::string>("frame").value();
    geometry_msgs::msg::TransformStamped transform = buffer.lookupTransform("world", objectName, tf2::TimePointZero, tf2::durationFromSec(1.0));
    geometry_msgs::msg::Vector3 translation = transform.transform.translation;

    double
        deltaY     = translation.y - currentPose.position.y,
        deltaX     = translation.x - currentPose.position.x,
        distance   = sqrt((deltaY * deltaY) + (deltaX * deltaX)),
        extraDistance = std::stod(getInput<std::string>("extra_distance").value()),
        distFactor = (distance + extraDistance) / distance;

    deltaY *= distFactor;
    deltaX *= distFactor;

    geometry_msgs::msg::Vector3 target;

    target.x = currentPose.position.x + deltaX;
    target.y = currentPose.position.y + deltaY;

    setOutput<std::string>("x_out", std::to_string(target.x));
    setOutput<std::string>("y_out", std::to_string(target.y));
    setOutput<std::string>("z_out", std::to_string(target.z));

    return NodeStatus::SUCCESS;
}


void LineDriveCalcState::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    currentPose = msg->pose.pose;
    odomExists = true;
}
