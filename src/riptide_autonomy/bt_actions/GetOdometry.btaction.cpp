#include "autonomy.h"

using namespace BT;
using std::placeholders::_1;

void GetOdometry::init(rclcpp::Node::SharedPtr node) { 
    this->rosnode = node;
    msgReceived = false;

    sub = rosnode->create_subscription<nav_msgs::msg::Odometry>(ODOMETRY_TOPIC, 10, std::bind(&GetOdometry::odomCallback, this, _1));
}


NodeStatus GetOdometry::tick() {
    rclcpp::Time startTime = rosnode->get_clock()->now();
    msgReceived = false;

    while(!msgReceived) {
        rclcpp::spin_some(rosnode);

        if((rosnode->get_clock()->now() - startTime).seconds() > 3) {
            RCLCPP_ERROR(log, "Timed out waiting for odometry.");
            return NodeStatus::FAILURE;
        }
    }

    //set linear position outputs
    setOutput<double>("x", odom.pose.pose.position.x);
    setOutput<double>("y", odom.pose.pose.position.y);
    setOutput<double>("z", odom.pose.pose.position.z);

    //convert quaternion to rpy and set outputs
    geometry_msgs::msg::Vector3 rpy = toRPY(odom.pose.pose.orientation);
    setOutput<double>("or", rpy.x);
    setOutput<double>("op", rpy.y);
    setOutput<double>("oy", rpy.z);

    return NodeStatus::SUCCESS;
}


void GetOdometry::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    msgReceived = true;
    odom = *msg;
}