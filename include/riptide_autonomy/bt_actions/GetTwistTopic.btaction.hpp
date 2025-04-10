#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

class GetTwistTopic : public UWRTActionNode {
    public:
    GetTwistTopic(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config) {
        
    }

    /**
     * @brief Declares ports needed by this node.
     * @return PortsList Needed ports.
     */
    static BT::PortsList providedPorts() {
        return {
            UwrtInput("topic", "Topic to listen to"),
            UwrtOutput("vel_x"),
            UwrtOutput("vel_y"),
            UwrtOutput("vel_z"),
            UwrtOutput("vel_roll"),
            UwrtOutput("vel_pitch"),
            UwrtOutput("vel_yaw")
        };
    }

    /**
     * @brief Initializes ROS peripherals such as publishers, subscribers, actions, services, etc.
     * Anything requiring the ROS node handle to construct should be initialized here. Do not do it in the 
     * constructor or you will be very sad
     */
    void rosInit() override { 

    }

    /**
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override {
        topic = tryGetRequiredInput<std::string>(this, "topic", "");
        if(topic == "")
        {
            RCLCPP_ERROR(rosNode()->get_logger(), "No topic given for GetTwistTopic");
            return BT::NodeStatus::FAILURE;
        }

        msgReceived = false;
        twistSub = rosNode()->create_subscription<geometry_msgs::msg::Twist>(
            topic,
            10,
            std::bind(&GetTwistTopic::twistCb, this, _1));
        
        startTime = rosNode()->get_clock()->now();

        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after 
     */
    BT::NodeStatus onRunning() override {
        if(msgReceived)
        {
            postOutput<double>(this, "vel_x", latestMsg.linear.x);
            postOutput<double>(this, "vel_y", latestMsg.linear.y);
            postOutput<double>(this, "vel_z", latestMsg.linear.z);
            postOutput<double>(this, "vel_roll", latestMsg.angular.x);
            postOutput<double>(this, "vel_pitch", latestMsg.angular.y);
            postOutput<double>(this, "vel_yaw", latestMsg.angular.z);
            return BT::NodeStatus::SUCCESS;
        }

        if(rosNode()->get_clock()->now() - startTime > 3s)
        {
            RCLCPP_ERROR(rosNode()->get_logger(), "Timed out waiting for Twist message on topic %s", topic.c_str());
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called when the node is halted.
     */
    void onHalted() override {

    }

    private:

    void twistCb(const geometry_msgs::msg::Twist::SharedPtr msg) {
        latestMsg = *msg;
        msgReceived = true;
    }

    std::string topic;
    geometry_msgs::msg::Twist latestMsg;
    bool msgReceived;
    rclcpp::Time startTime;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twistSub;
};
