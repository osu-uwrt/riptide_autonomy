#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

class WaitForDetection : public UWRTActionNode {
    public:
    WaitForDetection(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config) {
        
    }

    /**
     * @brief Declares ports needed by this node.
     * @return PortsList Needed ports.
     */
    static BT::PortsList providedPorts() {
        return {
            UwrtInput("object_name"),
            UwrtInput("timeout_secs")
        };
    }

    /**
     * @brief Initializes ROS peripherals such as publishers, subscribers, actions, services, etc.
     * Anything requiring the ROS node handle to construct should be initialized here. Do not do it in the 
     * constructor or you will be very sad
     */
    void rosInit() override { 
        sub = rosNode()->create_subscription<vision_msgs::msg::Detection3DArray>(
            DETECTIONS_TOPIC, 
            10,
            std::bind(&WaitForDetection::detection3dArrayCb, this, _1));
    }

    /**
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override {
        targetObjectId = tryGetRequiredInput<std::string>(this, "object_name", "");
        timeout = tryGetRequiredInput<double>(this, "timeout_secs", 0);
        foundObject = false;
        startTime = rosNode()->get_clock()->now();
        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after 
     */
    BT::NodeStatus onRunning() override {
        if(foundObject) {
            return BT::NodeStatus::SUCCESS;
        }

        if((rosNode()->get_clock()->now() - startTime).seconds() > timeout) {
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
    void detection3dArrayCb(const vision_msgs::msg::Detection3DArray::SharedPtr msg) {
        for(auto detection : msg->detections) {
            if(detection.results[0].hypothesis.class_id == targetObjectId) {
                foundObject = true;
            }
        }
    }

    bool foundObject = false;
    std::string targetObjectId;
    double timeout;
    rclcpp::Time startTime;
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr sub;
};
