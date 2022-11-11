#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

using namespace std::placeholders;
using AlignTorpedosMsg = riptide_msgs2::action::AlignTorpedos;

class AlignTorpedos : public UWRTActionNode {
    public:
    AlignTorpedos(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config) { }

    /**
     * @brief Declares ports needed by this node.
     * @return PortsList Needed ports.
     */
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("timeout"),
            BT::InputPort<double>("current_distance"),
            BT::InputPort<double>("goal_distance"),
            BT::OutputPort<double>("x"),
            BT::OutputPort<double>("y"),
            BT::OutputPort<double>("z")
        };
    }

    /**
     * @brief Initializes ROS peripherals such as publishers, subscribers, actions, services, etc.
     * Anything requiring the ROS node handle to construct should be initialized here. Do not do it in the 
     * constructor or you will be very sad
     */
    void rosInit() override {
        client = rclcpp_action::create_client<AlignTorpedosMsg>(this->rosnode, "AlignTorpedos");
        timeout = 10;
    }

    /**
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override {
        timeout = getInput<double>("timeout").value();

        double 
            currentDistance = getInput<double>("current_distance").value(),
            goalDistance = getInput<double>("goal_distance").value();
        
        if(!client->wait_for_action_server(std::chrono::duration<double>(timeout))) {
            RCLCPP_ERROR(log, "AlignTorpedos action server not available.");
            return BT::NodeStatus::FAILURE;
        }

        auto goal = AlignTorpedosMsg::Goal();
        goal.timeoutms = timeout * 1000;
        goal.currentdistance = currentDistance;
        goal.goaldistance = goalDistance;

        
        actionResult.code = rclcpp_action::ResultCode::UNKNOWN;
        auto options = rclcpp_action::Client<AlignTorpedosMsg>::SendGoalOptions();
        options.result_callback = std::bind(&AlignTorpedos::resultCB, this, _1);

        RCLCPP_INFO(log, "Sending goal to AlignTorpedos client.");
        auto future = client->async_send_goal(goal, options);

        this->startTime = rosnode->get_clock()->now();

        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after 
     */
    BT::NodeStatus onRunning() override {
        if(rclcpp::ok() && (rosnode->get_clock()->now() - startTime).seconds() < timeout && actionResult.code == rclcpp_action::ResultCode::UNKNOWN) {
            return BT::NodeStatus::RUNNING;
        }

        //at this point the result code is not UNKNOWN so we can return something
        switch(actionResult.code) {
            case rclcpp_action::ResultCode::SUCCEEDED: {
                setOutput<double>("x", actionResult.result->coords.x);
                setOutput<double>("y", actionResult.result->coords.y);
                setOutput<double>("z", actionResult.result->coords.z);
                return BT::NodeStatus::SUCCESS;
            }
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(log, "Torpedo Alignment aborted by server.");
                return BT::NodeStatus::FAILURE;
            default:
                RCLCPP_ERROR(log, "Torpedo alignment result either unknown or canceled. Likely timed out waiting for a response from the server.");
                return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::FAILURE;
    }

    /**
     * @brief Called when the node is halted.
     */
    void onHalted() override { }

    private:

    void resultCB(const rclcpp_action::ClientGoalHandle<AlignTorpedosMsg>::WrappedResult& result) {
        RCLCPP_INFO(log, "Action completed.");
        this->actionResult = result;
    }
    
    rclcpp_action::Client<AlignTorpedosMsg>::SharedPtr client;
    rclcpp_action::ClientGoalHandle<AlignTorpedosMsg>::WrappedResult actionResult;
    rclcpp::Time startTime;
    double timeout;
};
