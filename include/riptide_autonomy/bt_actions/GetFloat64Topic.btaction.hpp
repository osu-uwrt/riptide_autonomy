#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

class GetFloat64Topic : public UWRTActionNode {
    public:
    GetFloat64Topic(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config) {
        
    }

    /**
     * @brief Declares ports needed by this node.
     * @return PortsList Needed ports.
     */
    static BT::PortsList providedPorts() {
        return {
            UwrtInput("topic"),
            UwrtOutput("value")
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
        _topic = tryGetRequiredInput<std::string>(this, "topic", "");
        if(_topic.empty())
        {
            RCLCPP_ERROR(rosNode()->get_logger(), "GetFloat64Topic failing due to bad topic name");
            return BT::NodeStatus::FAILURE;
        }

        _hasData = false;
        _sub = rosNode()->create_subscription<std_msgs::msg::Float64>(_topic, 10, std::bind(&GetFloat64Topic::floatCb, this, _1));
        _startTime = rosNode()->get_clock()->now();
        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after 
     */
    BT::NodeStatus onRunning() override {
        rclcpp::Time now = rosNode()->get_clock()->now();

        // check if a message was received
        if(_hasData)
        {
            postOutput<double>(this, "value", _data);
            return BT::NodeStatus::SUCCESS;
        }
        
        // check if timed out
        if(now - _startTime > 3s)
        {
            RCLCPP_ERROR(rosNode()->get_logger(), "Timed out waiting for Float64 on topic %s", _topic.c_str());
            postOutput<bool>(this, "value", 0); // set a value on the blackboard so the rest of the tree doesnt crash if access is attempted
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
    
    void floatCb(const std_msgs::msg::Float64::SharedPtr msg)
    {
        _data = msg->data;
        _hasData = true;
    }


    bool _hasData;
    double _data;
    std::string _topic;
    rclcpp::Time _startTime;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _sub;
};
