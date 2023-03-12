#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

using ChangeClawState = riptide_msgs2::action::ChangeClawState;
using GoalHandleClaw = rclcpp_action::ClientGoalHandle<ChangeClawState>;
using ClawResult = rclcpp_action::ClientGoalHandle<ChangeClawState>::WrappedResult;
using namespace std::chrono_literals;
using namespace std::placeholders;

class ActuateClaw : public UWRTActionNode {
    public:
    ActuateClaw(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config) {
        
    }

    /**
     * @brief Declares ports needed by this node.
     * @return PortsList Needed ports.
     */
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("claw_open")
        };
    }

    /**
     * @brief Initializes ROS peripherals such as publishers, subscribers, actions, services, etc.
     * Anything requiring the ROS node handle to construct should be initialized here. Do not do it in the 
     * constructor or you will be very sad
     */
    void rosInit() override {
        client = rclcpp_action::create_client<ChangeClawState>(this->rosnode, CLAW_SERVER_NAME);
    }

    /**
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override {
        bool clawopen = tryGetRequiredInput<bool>(this, "claw_open", false);

        if(!client->wait_for_action_server(3s)) {
            RCLCPP_ERROR(log, "ActuateClaw action server not available.");
            return BT::NodeStatus::FAILURE;
        }

        auto goal = ChangeClawState::Goal();
        goal.clawopen = clawopen;

        actionResult.code = rclcpp_action::ResultCode::UNKNOWN;
        auto options = rclcpp_action::Client<ChangeClawState>::SendGoalOptions();
        options.goal_response_callback = std::bind(&ActuateClaw::goalResponseCallback, this, _1);
        options.result_callback = std::bind(&ActuateClaw::resultCB, this, _1);

        RCLCPP_INFO(log, "Sending goal to ActuateClaw client.");
        startTime = rosnode->get_clock()->now();
        auto future = client->async_send_goal(goal, options);

        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after 
     */
    BT::NodeStatus onRunning() override {
        if(rclcpp::ok() && (rosnode->get_clock()->now() - startTime) < 10s && actionResult.code == rclcpp_action::ResultCode::UNKNOWN) {
            return BT::NodeStatus::RUNNING;
        }

        if(actionResult.code == rclcpp_action::ResultCode::SUCCEEDED) {
            return BT::NodeStatus::SUCCESS;
        }

        RCLCPP_INFO(log, "Claw action failed! Returned code %i", (int) actionResult.code);
        return BT::NodeStatus::FAILURE;
    }

    /**
     * @brief Called when the node is halted.
     */
    void onHalted() override { }

    private:

    void goalResponseCallback(const GoalHandleClaw::SharedPtr & goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(log, "Claw actuation goal was rejected by server");
        } else {
            RCLCPP_INFO(log, "Claw actuation goal accepted by server, waiting for result");
        }
    }

    void resultCB(const ClawResult& result) {
        RCLCPP_INFO(log, "Action result received.");
        actionResult = result;
    }
    
    rclcpp::Time startTime;
    ClawResult actionResult;
    rclcpp_action::Client<ChangeClawState>::SharedPtr client;
};
