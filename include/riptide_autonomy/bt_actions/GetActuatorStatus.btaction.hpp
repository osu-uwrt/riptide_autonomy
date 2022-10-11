#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

using namespace std::placeholders;

class GetActuatorStatus : public UWRTActionNode {
    public:
    GetActuatorStatus(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config) { }

    /**
     * @brief Declares ports needed by this node.
     * @return PortsList Needed ports.
     */
    static BT::PortsList providedPorts() {
        return {
            BT::OutputPort<int>("claw_state"),
            BT::OutputPort<int>("torpedo1_state"),
            BT::OutputPort<int>("torpedo2_state"),
            BT::OutputPort<int>("dropper1_state"),
            BT::OutputPort<int>("dropper2_state")
        };
    }

    /**
     * @brief Initializes ROS peripherals such as publishers, subscribers, actions, services, etc.
     * Anything requiring the ROS node handle to construct should be initialized here. Do not do it in the 
     * constructor or you will be very sad
     */
    void rosInit() override { 
        statusSub = rosnode->create_subscription<riptide_msgs2::msg::ActuatorStatus> (
            ACTUATOR_STATUS_TOPIC, 
            rclcpp::SensorDataQoS(),
            std::bind(&GetActuatorStatus::statusCallback, this, _1)
        );

        statusReceived = false;
    }

    /**
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override {
        statusReceived = false; //...so that we force the node to collect another reading
        startTime = rosnode->get_clock()->now();

        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after 
     */
    BT::NodeStatus onRunning() override {
        if(!statusReceived && (rosnode->get_clock()->now() - startTime).seconds() > 3) {
            RCLCPP_ERROR(log, "Timed out getting actuator status.");
            return BT::NodeStatus::FAILURE;
        } else if(statusReceived) {
            setOutput<int>("claw_state", latestStatus.claw_state);
            setOutput<int>("torpedo1_state", latestStatus.torpedo1_state);
            setOutput<int>("torpedo2_state", latestStatus.torpedo2_state);
            setOutput<int>("dropper1_state", latestStatus.dropper1_state);
            setOutput<int>("dropper2_state", latestStatus.dropper2_state);
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
    void statusCallback(const riptide_msgs2::msg::ActuatorStatus::SharedPtr msg) {
        latestStatus = *msg;
        statusReceived = true;
    }

    bool statusReceived = false;
    riptide_msgs2::msg::ActuatorStatus latestStatus; 
    rclcpp::Subscription<riptide_msgs2::msg::ActuatorStatus>::SharedPtr statusSub;
    rclcpp::Time startTime;
};
