#include "autonomy.h"

using namespace BT;

void AlignTorpedos::init(rclcpp::Node::SharedPtr node) {
    this->rosnode = node;
}


NodeStatus AlignTorpedos::tick() {
    using AlignTorpedos = riptide_msgs2::action::AlignTorpedos;

    double 
        timeout = getInput<double>("timeout").value(),
        distance = getInput<double>("distance").value();

    auto client = rclcpp_action::create_client<AlignTorpedos>(this->rosnode, "AlignTorpedos");
    
    if(!client->wait_for_action_server(std::chrono::duration<double>(timeout))) {
        RCLCPP_ERROR(log, "AlignTorpedos action server not available.");
        return NodeStatus::FAILURE;
    }

    auto goal = AlignTorpedos::Goal();
    goal.timeoutms = timeout * 1000;
    goal.distance = distance;

    RCLCPP_INFO(log, "Sending goal to AlignTorpedos client.");
    auto future = client->async_send_goal(goal);

    //wait for action future to complete
    rclcpp::FutureReturnCode futureResult = rclcpp::spin_until_future_complete(this->rosnode, future, std::chrono::duration<double>(timeout));
    if(futureResult != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(log, "AlignTorpedos action did not return successfully.");
        return NodeStatus::FAILURE;
    }

    RCLCPP_INFO(log, "Attempting to grab AlignTorpedos result.");
    auto result = client->async_get_result(future.get()).get();
    RCLCPP_INFO(log, "Received alignment result.");

    switch(result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED: {
            setOutput<double>("x", result.result->coords.x);
            setOutput<double>("y", result.result->coords.y);
            setOutput<double>("z", result.result->coords.z);
            return NodeStatus::SUCCESS;
        }
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(log, "Torpedo Alignment aborted by server.");
            return NodeStatus::FAILURE;
        default:
            RCLCPP_ERROR(log, "Torpedo alignment result either unknown or canceled.");
            return NodeStatus::FAILURE;
    }

    return NodeStatus::FAILURE;
}
