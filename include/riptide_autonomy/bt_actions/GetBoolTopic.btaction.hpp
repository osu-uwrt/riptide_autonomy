#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

/**
 * This is here for now - may replace later with a generic subscription node for trivial types (int, double, string, bool)
 */
class GetBoolTopic : public UWRTActionNode {
    public:
    GetBoolTopic(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config) { }

    /**
     * @brief Declares ports needed by this node.
     * @return PortsList Needed ports.
     */
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("topic"),
            BT::OutputPort<bool>("value")
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
        topic = tryGetRequiredInput<std::string>(this, "topic", "/some_bool");
        sub = rosnode->create_subscription<std_msgs::msg::Bool>(
            topic,
            10,
            std::bind(&GetBoolTopic::boolCb, this, _1)
        );

        dataReceived = false;
        startTime = rosnode->get_clock()->now();
        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after 
     */
    BT::NodeStatus onRunning() override {
        if(dataReceived) {
            setOutput<bool>("value", data);
            return BT::NodeStatus::SUCCESS;
        }

        if(rosnode->get_clock()->now() - startTime > 3s) {
            RCLCPP_ERROR(log, "Timed out waiting for bool on topic %s!", topic.c_str());
            setOutput<bool>("value", false); // set a value on the blackboard so the rest of the tree doesnt crash if access is attempted
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
    void boolCb(const std_msgs::msg::Bool::SharedPtr msg) {
        data = msg->data;
        dataReceived = true;
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub;
    rclcpp::Time startTime;
    std::string topic;
    bool
        data,
        dataReceived;
};
