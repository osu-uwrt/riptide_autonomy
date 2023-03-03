#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

using namespace std::placeholders;

class GetSwitchState : public UWRTActionNode {
    public:
    GetSwitchState(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config) {
        
    }

    /**
     * @brief Declares ports needed by this node.
     * @return PortsList Needed ports.
     */
    static BT::PortsList providedPorts() {
        return {
            BT::OutputPort<bool>("kill_switch_inserted"),
            BT::OutputPort<bool>("aux_switch_inserted")
        };
    }

    /**
     * @brief Initializes ROS peripherals such as publishers, subscribers, actions, services, etc.
     * Anything requiring the ROS node handle to construct should be initialized here. Do not do it in the 
     * constructor or you will be very sad
     */
    void rosInit() override { 
        killedSub = rosnode->create_subscription<std_msgs::msg::Bool> (
            ROBOT_KILLED_TOPIC,
            rclcpp::SensorDataQoS(),
            std::bind(&GetSwitchState::killedCallback, this, _1)
        );

        auxSub = rosnode->create_subscription<std_msgs::msg::Bool> (
            ROBOT_AUX_TOPIC,
            rclcpp::SensorDataQoS(),
            std::bind(&GetSwitchState::auxCallback, this, _1)
        );
    }

    /**
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override {
        killedReceived = false; //force node to collect another message
        auxReceived = false;
        startTime = rosnode->get_clock()->now();
        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after 
     */
    BT::NodeStatus onRunning() override {
        if((rosnode->get_clock()->now() - startTime).seconds() > 3) {
            RCLCPP_ERROR(log, "Timed out getting switch state.");
            return BT::NodeStatus::FAILURE;
        } else if (killedReceived && auxReceived) {
            setOutput<bool>("kill_switch_inserted", latestKilledState.data);
            setOutput<bool>("aux_switch_inserted", latestAuxState.data);
            
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
    void killedCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        latestKilledState = *msg;
        killedReceived = true;
    }

    void auxCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        latestAuxState = *msg;
        auxReceived = true;
    }

    bool 
        killedReceived = false,
        auxReceived = false;
    
    std_msgs::msg::Bool
        latestKilledState,
        latestAuxState;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr 
        killedSub,
        auxSub;

    rclcpp::Time startTime;
};
