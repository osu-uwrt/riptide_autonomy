#include "bt_actions/ResetOdom.h"

using namespace BT;
using namespace std::chrono_literals;


PortsList ResetOdom::providedPorts() {
    return {
        BT::InputPort<double>("x"),
        BT::InputPort<double>("y"),
        BT::InputPort<double>("z"),
        BT::InputPort<double>("or"),
        BT::InputPort<double>("op"),
        BT::InputPort<double>("oy"),
    };
}


NodeStatus ResetOdom::tick() {
    auto client = rosnode->create_client<robot_localization::srv::SetPose>("/tempest/set_pose");

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

    // wait for the result
    result.wait();

    // should be valid now 
    if(! result.valid()){
        return NodeStatus::FAILURE;
    }
    return NodeStatus::SUCCESS;
}
