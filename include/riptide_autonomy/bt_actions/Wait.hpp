#pragma once

#include "riptide_autonomy/autonomy_base.hpp"
#include "riptide_autonomy/uwrt_node_types.hpp"

class Wait : public UWRTActionNode {
    public:
    Wait(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config) {
        goalTime = 0;
    }

    /**
     * @brief Declares ports needed by this node.
     * @return PortsList Needed ports.
     */
    static UwrtPortInformation portInformation() {
        return {
            UwrtInput("seconds", PORT_REQUIRED,
                "Seconds to wait while spinning ROS node")
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
        startTime = rosNode()->get_clock()->now();
        goalTime = tryGetRequiredInput<double>("seconds", 0);
        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after 
     */
    BT::NodeStatus onRunning() override {
        auto timeElapsed = rosNode()->get_clock()->now() - startTime;
        return (timeElapsed.seconds() >= goalTime ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING);
    }

    /**
     * @brief Called when the node is halted.
     */
    void onHalted() override {

    }

    private:
    rclcpp::Time startTime;
    double goalTime;
};
