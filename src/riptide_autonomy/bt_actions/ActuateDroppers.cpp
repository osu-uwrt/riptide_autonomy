#include "autonomy.h"

using namespace BT;

using namespace std::chrono_literals;


void ActuateDroppers::init(rclcpp::Node::SharedPtr node) { 
    this->rosnode = node;
}


NodeStatus ActuateDroppers::tick() { 
    using ActuateDroppers = riptide_msgs2::action::ActuateDroppers;

    int dropperID = getInput<int>("DropperID").value();

    auto client = rclcpp_action::create_client<ActuateDroppers>(this->rosnode, "/dropper");

    if(!client->wait_for_action_server(10s)) {
        RCLCPP_ERROR(log, "ActuateDroppers action server not available.");
        return NodeStatus::FAILURE;
    }

    auto goal = ActuateDroppers::Goal();
    goal.dropper_id = dropperID;

    rclcpp_action::ClientGoalHandle<ActuateDroppers>::WrappedResult actionResult;
    actionResult.code = rclcpp_action::ResultCode::UNKNOWN;
    auto options = rclcpp_action::Client<ActuateDroppers>::SendGoalOptions();
    options.result_callback = 
        [&actionResult] (const rclcpp_action::ClientGoalHandle<ActuateDroppers>::WrappedResult& result) {
            RCLCPP_INFO(log, "Action completed.");
            actionResult = result;
        };
    

    RCLCPP_INFO(log, "Sending goal to ActuateDroppers client.");
    auto future = client->async_send_goal(goal, options);

    rclcpp::Time startTime = rosnode->get_clock()->now();
    while(rclcpp::ok() && actionResult.code == rclcpp_action::ResultCode::UNKNOWN && (rosnode->get_clock()->now() - startTime) < 10s){
        rclcpp::spin_some(rosnode);
    }

    switch(actionResult.code) {
        case rclcpp_action::ResultCode::SUCCEEDED: {
            return NodeStatus::SUCCESS;
        }
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(log, "Actuate droppers action aborted by server.");
            return NodeStatus::FAILURE;
        default:
            RCLCPP_ERROR(log, "Actuate droppers action either unknown or canceled. Likely timed out waiting for a response from the server.");
            return NodeStatus::FAILURE;
    }

    return NodeStatus::FAILURE;
}
