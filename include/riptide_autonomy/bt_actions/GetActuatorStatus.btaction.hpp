#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

class GetActuatorStatus : public UWRTActionNode {
    public:
    GetActuatorStatus(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config) {
        
    }

    /**
     * @brief Declares ports needed by this node.
     * @return PortsList Needed ports.
     */
    static BT::PortsList providedPorts() {
        return {
            UwrtOutput("claw_state"),
            UwrtOutput("torpedo_state"),
            UwrtOutput("torpedo_available_count"),
            UwrtOutput("dropper_state"),
            UwrtOutput("dropper_available_count"),
            UwrtOutput("actuators_busy")
        };
    }

    /**
     * @brief Initializes ROS peripherals such as publishers, subscribers, actions, services, etc.
     * Anything requiring the ROS node handle to construct should be initialized here. Do not do it in the 
     * constructor or you will be very sad
     */
    void rosInit() override { 
        statusSub = rosnode->create_subscription<riptide_msgs2::msg::ActuatorStatus>(
            ACTUATOR_STATUS_TOPIC,
            rclcpp::SensorDataQoS(),
            std::bind(&GetActuatorStatus::statusCb, this, _1)
        );

        busySub = rosnode->create_subscription<std_msgs::msg::Bool>(
            ACTUATOR_BUSY_TOPIC,
            rclcpp::SensorDataQoS(),
            std::bind(&GetActuatorStatus::busyCb, this, _1)
        );
    }

    /**
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override {
        statusReceived = false;
        busyReceived = false;
        startTime = rosnode->get_clock()->now();

        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after 
     */
    BT::NodeStatus onRunning() override {
        //have we timed out yet?
        if(rosnode->get_clock()->now() - startTime > 3s) {
            RCLCPP_ERROR(rosNode()->get_logger(), "Timed out waiting for full actuator status. Status received: %d, busy received: %d", statusReceived, busyReceived);
            return BT::NodeStatus::FAILURE;
        }

        //have we gotten the messages we need?
        if(statusReceived && busyReceived) {
            postOutput<int>(this, "claw_state", latestStatus.claw_state);
            postOutput<int>(this, "torpedo_state", latestStatus.torpedo_state);
            postOutput<int>(this, "torpedo_available_count", latestStatus.torpedo_available_count);
            postOutput<int>(this, "dropper_state", latestStatus.dropper_state);
            postOutput<int>(this, "dropper_available_count", latestStatus.dropper_available_count);
            postOutput<bool>(this, "actuators_busy", latestBusy.data);
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
    bool
        statusReceived,
        busyReceived;

    riptide_msgs2::msg::ActuatorStatus latestStatus;
    std_msgs::msg::Bool latestBusy;

    rclcpp::Subscription<riptide_msgs2::msg::ActuatorStatus>::SharedPtr statusSub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr busySub;

    rclcpp::Time startTime;

    void statusCb(const riptide_msgs2::msg::ActuatorStatus::SharedPtr msg) {
        latestStatus = *msg;
        statusReceived = true;
    }

    void busyCb(const std_msgs::msg::Bool::SharedPtr msg) {
        latestBusy = *msg;
        busyReceived = true;
    }
};
