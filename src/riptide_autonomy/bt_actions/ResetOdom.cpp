#include "autonomy.h"

using namespace BT;
using namespace std::chrono_literals;

void ResetOdom::init(rclcpp::Node::SharedPtr node) {
    this->rosnode = node;
    
    this->client = rosnode->create_client<robot_localization::srv::SetPose>("/set_pose");
}

NodeStatus ResetOdom::tick() {
    //get inputs from BT
    auto request = std::make_shared<robot_localization::srv::SetPose::Request>();

    request->pose.pose.pose.position.x = getInput<double>("x").value();
    request->pose.pose.pose.position.y = getInput<double>("y").value();
    request->pose.pose.pose.position.z = getInput<double>("z").value();

    geometry_msgs::msg::Vector3 orient;
    orient.x = getInput<double>("or").value();
    orient.y = getInput<double>("op").value();
    orient.z = getInput<double>("oy").value();

    request->pose.pose.pose.orientation = toQuat(orient);

    while(! client->wait_for_service(1s)){
        if(! rclcpp::ok()) return NodeStatus::FAILURE;
    }

    auto result = client->async_send_request(request);

    if(rclcpp::spin_until_future_complete(rosnode, result) != rclcpp::FutureReturnCode::SUCCESS){
        return NodeStatus::FAILURE;
    }
    return NodeStatus::SUCCESS;
}
