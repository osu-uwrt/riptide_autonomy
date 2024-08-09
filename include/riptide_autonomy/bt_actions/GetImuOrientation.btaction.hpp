#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

class GetImuOrientation : public UWRTActionNode {
    public:
    GetImuOrientation(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config) {
        
    }

    /**
     * @brief Declares ports needed by this node.
     * @return PortsList Needed ports.
     */
    static BT::PortsList providedPorts() {
        return {
            UwrtInput("topic"),
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

    }

    /**
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override {
        std::string topic = tryGetRequiredInput<std::string>(this, "topic", "");
        if(topic == "")
        {
            RCLCPP_ERROR(rosNode()->get_logger(), "No topic provided");
            return BT::NodeStatus::FAILURE;
        }

        sub = rosNode()->create_subscription<sensor_msgs::msg::Imu>(
            topic,
            10,
            std::bind(&GetImuOrientation::msgCb, this, _1));
        
        receivedMsg = false;
        startTime = rosNode()->get_clock()->now();
        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after 
     */
    BT::NodeStatus onRunning() override {
        //check if received
        if(!receivedMsg)
        {
            //check if timed out
            if(rosNode()->get_clock()->now() - startTime > 3s)
            {
                RCLCPP_ERROR(rosNode()->get_logger(), "Failed to receive quaternion on topic %s", sub->get_topic_name());
                return BT::NodeStatus::FAILURE;
            }

            //still going
            return BT::NodeStatus::RUNNING;
        }

        //got msg! pack and succeed
        postOutput<double>(this, "or", lastRpy.x);
        postOutput<double>(this, "op", lastRpy.y);
        postOutput<double>(this, "oy", lastRpy.z);
        return BT::NodeStatus::SUCCESS;
    }

    /**
     * @brief Called when the node is halted.
     */
    void onHalted() override {

    }

    private:
    void msgCb(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        lastRpy = toRPY(msg->orientation);
        receivedMsg = true;
    }

    rclcpp::Time startTime;
    bool receivedMsg;
    geometry_msgs::msg::Vector3 lastRpy;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub;
};
