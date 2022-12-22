#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

using namespace std::chrono_literals;

class ResetOdom : public UWRTActionNode {
    public:
    ResetOdom(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config) {
        
    }

    /**
     * @brief Declares ports needed by this node.
     * @return PortsList Needed ports.
     */
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("x"),
            BT::InputPort<double>("y"),
            BT::InputPort<double>("z"),
            BT::InputPort<double>("or"),
            BT::InputPort<double>("op"),
            BT::InputPort<double>("oy"),
        };
    }

    /**
     * @brief Initializes ROS peripherals such as publishers, subscribers, actions, services, etc.
     * Anything requiring the ROS node handle to construct should be initialized here. Do not do it in the 
     * constructor or you will be very sad
     */
    void rosInit() override { 
        client = rosnode->create_client<robot_localization::srv::SetPose>("/tempest/set_pose");
    }

    /**
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override {
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
            if(! rclcpp::ok()) return BT::NodeStatus::FAILURE;
        }

        future = client->async_send_request(request);
        startTime = rosnode->get_clock()->now();

        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after 
     */
    BT::NodeStatus onRunning() override {
        // should be valid now 
        if(!future.valid() && rosnode->get_clock()->now() - startTime > 5s){
            return BT::NodeStatus::FAILURE;
        } else if (future.valid()) {
            return BT::NodeStatus::SUCCESS;
        }
        
        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called when the node is halted.
     */
    void onHalted() override {

    }

    private:
    rclcpp::Client<robot_localization::srv::SetPose>::SharedPtr client;
    rclcpp::Client<robot_localization::srv::SetPose>::SharedFuture future;
    rclcpp::Time startTime;
};
