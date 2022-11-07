#include "autonomy.h"

using namespace BT;
using namespace std::chrono_literals;

void ActuateClaw::init(rclcpp::Node::SharedPtr node) { 
    this->rosnode = node;

}

NodeStatus ActuateClaw::tick() { 
    using ActuateClaw = riptide_msgs2::action::ChangeClawState;

    bool
        clawopen = getInput<bool>("claw_open").value();

    auto client = rclcpp_action::create_client<ActuateClaw>(this->rosnode, "claw");

    if(!client->wait_for_action_server(3s)) {
        RCLCPP_ERROR(log, "ActuateClaw action server not available.");
        return NodeStatus::FAILURE;
    }

    auto goal = ActuateClaw::Goal();
    goal.clawopen = clawopen;

    rclcpp_action::ClientGoalHandle<ActuateClaw>::WrappedResult actionResult;
    actionResult.code = rclcpp_action::ResultCode::UNKNOWN;
    auto options = rclcpp_action::Client<ActuateClaw>::SendGoalOptions();
    options.result_callback = 
        [&actionResult] (const rclcpp_action::ClientGoalHandle<ActuateClaw>::WrappedResult& result) {
            RCLCPP_INFO(log, "Action completed.");
            actionResult = result;
        };
    

    RCLCPP_INFO(log, "Sending goal to AlignTorpedos client.");
    auto future = client->async_send_goal(goal, options);

    rclcpp::Time startTime = rosnode->get_clock()->now();
    while(rclcpp::ok() && actionResult.code == rclcpp_action::ResultCode::UNKNOWN && (rosnode->get_clock()->now() - startTime) < 3s) {
        rclcpp::spin_some(rosnode);
    }

switch(actionResult.code) {
        case rclcpp_action::ResultCode::SUCCEEDED: {
            return NodeStatus::SUCCESS;
        }
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(log, "Accuate Claw aborted by server.");
            return NodeStatus::FAILURE;
        default:
            RCLCPP_ERROR(log, "Accuate Claw result either unknown or canceled. Likely timed out waiting for a response from the server.");
            return NodeStatus::FAILURE;
    }

    return NodeStatus::FAILURE;
}
