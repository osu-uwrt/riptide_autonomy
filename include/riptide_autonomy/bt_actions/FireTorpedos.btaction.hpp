#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

using ChangeTorpedoState = riptide_msgs2::action::ActuateTorpedos;
using GoalHandleTorpedo = rclcpp_action::ClientGoalHandle<ChangeTorpedoState>;
using TorpedoResult = rclcpp_action::ClientGoalHandle<ChangeTorpedoState>::WrappedResult;
using namespace std::chrono_literals;
using namespace std::placeholders;

class FireTorpedos : public UWRTActionNode {
    public:
    FireTorpedos(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config) {
        
    }

    /**
     * @brief Declares ports needed by this node.
     * @return PortsList Needed ports.
     */
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<int>("TorpedoID")
        };
    }

    /**
     * @brief Initializes ROS peripherals such as publishers, subscribers, actions, services, etc.
     * Anything requiring the ROS node handle to construct should be initialized here. Do not do it in the 
     * constructor or you will be very sad
     */
    void rosInit() override { 
        client = rclcpp_action::create_client<ChangeTorpedoState>(this->rosnode, TORPEDO_SERVER_NAME);
    }

    /**
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override {
        int torpedoID = tryGetRequiredInput<int>(this, "TorpedoID", -1); 

        if(!client->wait_for_action_server(3s)) {
            RCLCPP_ERROR(log, "FireTorpedos action server not available.");
            return BT::NodeStatus::FAILURE;
        }

        auto goal = ChangeTorpedoState::Goal();
        goal.torpedo_id = torpedoID;

        actionResult.code = rclcpp_action::ResultCode::UNKNOWN;
        auto options = rclcpp_action::Client<ChangeTorpedoState>::SendGoalOptions();
        options.goal_response_callback = std::bind(&FireTorpedos::goalResponseCB, this, _1);
        options.result_callback = std::bind(&FireTorpedos::resultCB, this, _1);

        startTime = rosnode->get_clock()->now();
        auto future = client->async_send_goal(goal, options);
        goalAccepted = true;

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

        RCLCPP_INFO(log, "Could not fire the torpedos! Returned code %i", (int) actionResult.code);
        return BT::NodeStatus::FAILURE;
    }

    /**
     * @brief Called when the node is halted.
     */
    void onHalted() override { } 

    private:

    void goalResponseCB(const GoalHandleTorpedo::SharedPtr& goalHandle) {
        if(!goalHandle) {
            RCLCPP_ERROR(log, "Could not fire the torpedos, the goal was rejected by the server!");
            goalAccepted = false;
        } else {
            RCLCPP_INFO(log, "Torpedo action accepted by server; waiting for result");
        }
    }

    void resultCB(const TorpedoResult& result) {
        RCLCPP_INFO(log, "Action completed.");
        actionResult = result;
    }
    
    bool goalAccepted = true;
    rclcpp::Time startTime;
    TorpedoResult actionResult;
    rclcpp_action::Client<ChangeTorpedoState>::SharedPtr client;
};
