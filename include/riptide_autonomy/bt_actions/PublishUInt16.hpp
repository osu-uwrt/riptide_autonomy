#pragma once

#include "riptide_autonomy/autonomy_base.hpp"
#include "riptide_autonomy/uwrt_node_types.hpp"

#include <std_msgs/msg/u_int16.hpp>

class PublishUInt16 : public UWRTActionNode {
    public:
    PublishUInt16(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config) {
        
    }

    /**
     * @brief Declares ports needed by this node.
     * @return PortsList Needed ports.
     */
    static BT::PortsList providedPorts() {
        return {
            UwrtInput("topic", PORT_REQUIRED,
                "Name of the topic to publish to"),

            UwrtInput("data", PORT_REQUIRED,
                "Data to publish to topic")
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
        std::string topic = tryGetRequiredInput<std::string>("topic", "");
        if(topic == "") {
            return BT::NodeStatus::FAILURE;
        }

        int data = tryGetRequiredInput<int>("data", 0);

        pub = rosNode()->create_publisher<std_msgs::msg::UInt16>(topic, 10);
        std_msgs::msg::UInt16 msg;
        msg.data = data;
        pub->publish(msg);

        return BT::NodeStatus::SUCCESS;
    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after 
     */
    BT::NodeStatus onRunning() override {
        return BT::NodeStatus::SUCCESS;
    }

    /**
     * @brief Called when the node is halted.
     */
    void onHalted() override {

    }

    private:
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pub;
};
