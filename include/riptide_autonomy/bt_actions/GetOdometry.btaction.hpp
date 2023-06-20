#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

using namespace std::placeholders;

class GetOdometry : public UWRTActionNode {
    public:
    GetOdometry(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config) { }

    /**
     * @brief Declares ports needed by this node.
     * @return PortsList Needed ports.
     */
    static BT::PortsList providedPorts() {
        return {
            UwrtOutput("x"),
            UwrtOutput("y"),
            UwrtOutput("z"),
            UwrtOutput("or"),
            UwrtOutput("op"),
            UwrtOutput("oy")
        };
    }

    /**
     * @brief Initializes ROS peripherals such as publishers, subscribers, actions, services, etc.
     * Anything requiring the ROS node handle to construct should be initialized here. Do not do it in the 
     * constructor or you will be very sad
     */
    void rosInit() override { 
        sub = rosnode->create_subscription<nav_msgs::msg::Odometry>(
            ODOMETRY_TOPIC, 
            10, 
            std::bind(&GetOdometry::odomCallback, this, _1)
        );
    }

    /**
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override {
        msgReceived = false; //force node to collect another message
        startTime = rosnode->get_clock()->now();

        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after 
     */
    BT::NodeStatus onRunning() override {
        if(!msgReceived && (rosnode->get_clock()->now() - startTime).seconds() > 3) {
            RCLCPP_ERROR(log, "Timed out waiting for odometry.");
            return BT::NodeStatus::FAILURE;
        } else if(msgReceived) {
            //set linear position outputs
            postOutput<double>(this, "x", odom.pose.pose.position.x);
            postOutput<double>(this, "y", odom.pose.pose.position.y);
            postOutput<double>(this, "z", odom.pose.pose.position.z);

            //convert quaternion to rpy and set outputs
            geometry_msgs::msg::Vector3 rpy = toRPY(odom.pose.pose.orientation);
            postOutput<double>(this, "or", rpy.x);
            postOutput<double>(this, "op", rpy.y);
            postOutput<double>(this, "oy", rpy.z);

            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called when the node is halted.
     */
    void onHalted() override {

    }

    private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        msgReceived = true;
        odom = *msg;
    }

    bool msgReceived = false;
    nav_msgs::msg::Odometry odom;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub;
    rclcpp::Time startTime;
};
