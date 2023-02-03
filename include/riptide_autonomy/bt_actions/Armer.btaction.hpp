#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

using ArmTorpedoDropperMSG = riptide_msgs2::action::ArmTorpedoDropper;
using ArmerGoalHandle = rclcpp_action::ClientGoalHandle<ArmTorpedoDropperMSG>;

class Armer : public UWRTActionNode {
    public:
    Armer(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config) {
        
    }

    /**
     * @brief Declares ports needed by this node.
     * @return PortsList Needed ports.
     */
    static BT::PortsList providedPorts() {
        return {
             BT::InputPort<bool>("ArmTorpedos"),
             BT::InputPort<bool>("ArmDroppers"),
             BT::OutputPort<bool>("IsArmed")

        };
    }

    /**
     * @brief Initializes ROS peripherals such as publishers, subscribers, actions, services, etc.
     * Anything requiring the ROS node handle to construct should be initialized here. Do not do it in the 
     * constructor or you will be very sad
     */
    void rosInit() override { 
        client = rclcpp_action::create_client<ArmTorpedoDropperMSG>(this->rosnode, "arm_torpedo_dropper");
    }

    /**
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override {

        if(!client->wait_for_action_server(3s)) {
            RCLCPP_ERROR(log, "ActuateArmer action server not available.");
            return BT::NodeStatus::FAILURE;
        }

        bool arm_droppers = getInput<bool>("ArmDroppers").value();
        bool arm_torpedos = getInput<bool>("ArmTorpedos").value();


        auto goal = ArmTorpedoDropperMSG::Goal();
        goal.arm_droppers = arm_droppers;
        goal.arm_torpedos = arm_torpedos;



        auto options = rclcpp_action::Client<ArmTorpedoDropperMSG>::SendGoalOptions();
        options.goal_response_callback = std::bind(&Armer::goalResponseCB, this, _1);
        options.feedback_callback = std::bind(&Armer::feedbackCB, this, _1, _2);        
        options.result_callback = std::bind(&Armer::resultCB, this, _1);
 
        if(isArmed && (arm_droppers || arm_torpedos)){
            RCLCPP_INFO(log, "Something is already armed please disarm first");
            return BT::NodeStatus::FAILURE;
        }
        if(arm_droppers && arm_torpedos){
            RCLCPP_INFO(log, "Only arm one thing at a time");
            return BT::NodeStatus::FAILURE;
        }

        startTime = rosnode->get_clock()->now();
        auto future = client->async_send_goal(goal, options);

        if(future.get() == nullptr) {
            RCLCPP_ERROR(log, "Could not arm, the goal was rejected by the server!");
            return BT::NodeStatus::FAILURE;
        }

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
        if(actionResult.code == rclcpp_action::ResultCode::ABORTED){
            RCLCPP_INFO(log, "Action was aborted. Could not arm.");
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(log, "Armer action failed! Returned code %i", (int) actionResult.code);
        return BT::NodeStatus::FAILURE;
    }

    /**
     * @brief Called when the node is halted.
     */
    void onHalted() override {
        client->async_cancel_all_goals();
    }
    private:
 void goalResponseCB(const ArmerGoalHandle::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
        RCLCPP_ERROR(log, "Goal was rejected by server");
    } else {
        RCLCPP_INFO(log, "Goal accepted by server, waiting for result");
    }
  }

    void feedbackCB(ArmerGoalHandle::SharedPtr, const std::shared_ptr<const ArmTorpedoDropperMSG::Feedback> feedback){
        isArmed = feedback->is_armed;
    }
    void resultCB(const ArmerGoalHandle::WrappedResult& result) {
        actionResult = result;
    }
    rclcpp_action::Client<ArmTorpedoDropperMSG>::SharedPtr client;
    ArmerGoalHandle::WrappedResult actionResult;
    bool isArmed;
    rclcpp::Time startTime;

};
