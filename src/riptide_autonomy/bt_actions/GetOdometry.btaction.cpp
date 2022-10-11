#include "bt_actions/GetOdometry.h"

using namespace BT;
using std::placeholders::_1;




PortsList GetOdometry::providedPorts() {
    return {
        BT::OutputPort<double>("x"),
        BT::OutputPort<double>("y"),
        BT::OutputPort<double>("z"),
        BT::OutputPort<double>("or"),
        BT::OutputPort<double>("op"),
        BT::OutputPort<double>("oy")
    };
}


static void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    msgReceived = true;
    odom = *msg;
}

NodeStatus GetOdometry::tick() {
    bool msgReceived = false; //force node to collect another message

    auto sub = rosnode->create_subscription<nav_msgs::msg::Odometry>(ODOMETRY_TOPIC, 10, std::bind(&odomCallback, _1));

    rclcpp::Time startTime = rosnode->get_clock()->now();
    msgReceived = false;

    while(!msgReceived) {
        // rclcpp::spin_some(rosnode);

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
