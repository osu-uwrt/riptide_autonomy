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
        stateSub = rosnode->create_subscription<riptide_msgs2::msg::RobotState> (
            ROBOT_STATE_TOPIC,
            rclcpp::SensorDataQoS(),
            std::bind(&GetSwitchState::robotStateCallback, this, _1)
        );
    }

    /**
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override {
        stateReceived = false; //force node to collect another message
        startTime = rosnode->get_clock()->now();

        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after 
     */
    BT::NodeStatus onRunning() override {
        if(!stateReceived && (rosnode->get_clock()->now() - startTime).seconds() > 3) {
            RCLCPP_ERROR(log, "Timed out getting switch state.");
            return BT::NodeStatus::FAILURE;
        } else if (stateReceived) {
            setOutput<bool>("kill_switch_inserted", latestState.kill_switch_inserted);
            setOutput<bool>("aux_switch_inserted", latestState.aux_switch_inserted);
            
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
    void robotStateCallback(const riptide_msgs2::msg::RobotState::SharedPtr msg) {
        latestState = *msg;
        stateReceived = true;
    }

    bool stateReceived = false;
    riptide_msgs2::msg::RobotState latestState;
    rclcpp::Subscription<riptide_msgs2::msg::RobotState>::SharedPtr stateSub;
    rclcpp::Time startTime;
};
