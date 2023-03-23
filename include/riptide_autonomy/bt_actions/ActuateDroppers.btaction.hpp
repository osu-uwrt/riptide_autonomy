#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

using ChangeDropperState = riptide_msgs2::action::ActuateDroppers;
using GoalHandleDroppers = rclcpp_action::ClientGoalHandle<ChangeDropperState>;
using DroppersResult = rclcpp_action::ClientGoalHandle<ChangeDropperState>::WrappedResult;
using namespace std::chrono_literals;
using namespace std::placeholders;

class ActuateDroppers : public UWRTActionNode {
    public:
    ActuateDroppers(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config) { }

    /**
     * @brief Declares ports needed by this node.
     * @return PortsList Needed ports.
     */
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<int>("DropperID")
        };
    }

    /**
     * @brief Initializes ROS peripherals such as publishers, subscribers, actions, services, etc.
     * Anything requiring the ROS node handle to construct should be initialized here. Do not do it in the 
     * constructor or you will be very sad
     */
    void rosInit() override {
        client = rclcpp_action::create_client<ChangeDropperState>(this->rosnode, DROPPER_SERVER_NAME);
    }

    /**
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override {
        int dropperID = tryGetRequiredInput<int>(this, "DropperID", -1); 

        if(!client->wait_for_action_server(3s)) {
            RCLCPP_ERROR(log, "ActuateDroppers action server not available.");
            return BT::NodeStatus::FAILURE;
        }

        auto goal = ChangeDropperState::Goal();
        goal.dropper_id = dropperID;

        actionResult.code = rclcpp_action::ResultCode::UNKNOWN;
        auto options = rclcpp_action::Client<ChangeDropperState>::SendGoalOptions();
        options.goal_response_callback = std::bind(&ActuateDroppers::goalResponseCallback, this, _1);
        options.result_callback = std::bind(&ActuateDroppers::resultCB, this, _1);

        RCLCPP_INFO(log, "Sending goal to ActuateDroppers server.");
        startTime = rosnode->get_clock()->now();
        goalAccepted = true;
        auto future = client->async_send_goal(goal, options);

        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after 
     */
    BT::NodeStatus onRunning() override {
        if( goalAccepted 
            && rclcpp::ok() 
            && (rosnode->get_clock()->now() - startTime) < 10s 
            && actionResult.code == rclcpp_action::ResultCode::UNKNOWN) 
        {
            return BT::NodeStatus::RUNNING;
        }

        if(actionResult.code == rclcpp_action::ResultCode::SUCCEEDED) {
            return BT::NodeStatus::SUCCESS;
        }

        RCLCPP_INFO(log, "Dropper action failed! Returned code %i", (int) actionResult.code);
        return BT::NodeStatus::FAILURE;
    }

    /**
     * @brief Called when the node is halted.
     */
    void onHalted() override { }

    private:

    void goalResponseCallback(const GoalHandleDroppers::SharedPtr& goal_handle) {
        if(!goal_handle) {
            RCLCPP_ERROR(log, "Dropper goal was rejected by the server");
            goalAccepted = false;
        } else {
            RCLCPP_INFO(log, "Dropper goal accepted by the server, waiting for result");
        }
    }

    void resultCB(const DroppersResult& result) {
        actionResult = result;
    }

    bool goalAccepted;
    rclcpp::Time startTime;
    DroppersResult actionResult;
    rclcpp_action::Client<ChangeDropperState>::SharedPtr client;
};
