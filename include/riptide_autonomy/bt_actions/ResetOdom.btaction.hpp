#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

using namespace std::chrono_literals;

class ResetOdom : public UWRTActionNode {
    using SetPose = robot_localization::srv::SetPose;

    public:
    ResetOdom(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config),
      result(std::future<std::shared_ptr<SetPose::Response>>(), 0) {
        
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
        client = rosnode->create_client<SetPose>("/talos/set_pose");
    }

    /**
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override {
        //get inputs from BT
        auto request = std::make_shared<SetPose::Request>();

        request->pose.pose.pose.position.x = tryGetRequiredInput<double>(this, "x", 0);
        request->pose.pose.pose.position.y = tryGetRequiredInput<double>(this, "y", 0);
        request->pose.pose.pose.position.z = tryGetRequiredInput<double>(this, "z", 0);

        geometry_msgs::msg::Vector3 orient;
        orient.x = tryGetRequiredInput<double>(this, "or", 0);
        orient.y = tryGetRequiredInput<double>(this, "op", 0);
        orient.z = tryGetRequiredInput<double>(this, "oy", 0);

        request->pose.pose.pose.orientation = toQuat(orient);

        while(! client->wait_for_service(1s)){
            RCLCPP_ERROR(log, "ResetOdom service is not available.");
            return BT::NodeStatus::FAILURE;
        }

        result = client->async_send_request(request);
        startTime = rosnode->get_clock()->now();

        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after 
     */
    BT::NodeStatus onRunning() override {
        // should be valid now 
        if(!result.valid() && rosnode->get_clock()->now() - startTime > 5s){
            return BT::NodeStatus::FAILURE;
        } else if (result.wait_for(0s) == std::future_status::ready) {
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
    rclcpp::Client<SetPose>::SharedPtr client;
    rclcpp::Client<SetPose>::FutureAndRequestId result;
    rclcpp::Time startTime;
};
