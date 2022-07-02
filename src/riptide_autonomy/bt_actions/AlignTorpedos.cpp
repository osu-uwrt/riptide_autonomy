#include "autonomy.h"

using namespace BT;

void AlignTorpedos::init(rclcpp::Node::SharedPtr node) {
    this->rosnode = node;
}


NodeStatus AlignTorpedos::tick() {
    using AlignTorpedos = riptide_msgs2::action::AlignTorpedos;

    double 
        timeout = getInput<double>("timeout").value(),
        currentDistance = getInput<double>("current_distance").value(),
        goalDistance = getInput<double>("goal_distance").value();

    auto client = rclcpp_action::create_client<AlignTorpedos>(this->rosnode, "AlignTorpedos");
    
    if(!client->wait_for_action_server(std::chrono::duration<double>(timeout))) {
        RCLCPP_ERROR(log, "AlignTorpedos action server not available.");
        return NodeStatus::FAILURE;
    }

    auto goal = AlignTorpedos::Goal();
    goal.timeoutms = timeout * 1000;
    //goal.currentdistance = currentDistance;
    //goal.goaldistance = goalDistance;

    rclcpp_action::ClientGoalHandle<AlignTorpedos>::WrappedResult actionResult;
    actionResult.code = rclcpp_action::ResultCode::UNKNOWN;
    auto options = rclcpp_action::Client<AlignTorpedos>::SendGoalOptions();
    options.result_callback = 
        [&actionResult] (const rclcpp_action::ClientGoalHandle<AlignTorpedos>::WrappedResult& result) {
            RCLCPP_INFO(log, "Action completed.");
            actionResult = result;
        };
    

    RCLCPP_INFO(log, "Sending goal to AlignTorpedos client.");
    auto future = client->async_send_goal(goal, options);

    rclcpp::Time startTime = rosnode->get_clock()->now();
    while(rclcpp::ok() && actionResult.code == rclcpp_action::ResultCode::UNKNOWN && (rosnode->get_clock()->now() - startTime).seconds() < timeout) {
        rclcpp::spin_some(rosnode);
    }

    switch(actionResult.code) {
        case rclcpp_action::ResultCode::SUCCEEDED: {
            setOutput<double>("x", actionResult.result->coords.x);
            setOutput<double>("y", actionResult.result->coords.y);
            setOutput<double>("z", actionResult.result->coords.z);
            return NodeStatus::SUCCESS;
        }
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(log, "Torpedo Alignment aborted by server.");
            return NodeStatus::FAILURE;
        default:
            RCLCPP_ERROR(log, "Torpedo alignment result either unknown or canceled. Likely timed out waiting for a response from the server.");
            return NodeStatus::FAILURE;
    }

    return NodeStatus::FAILURE;
}
