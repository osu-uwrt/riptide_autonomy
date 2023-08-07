#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

class CallSetBoolService : public UWRTActionNode {
    using SetBool = std_srvs::srv::SetBool;

    public:
    CallSetBoolService(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config), 
      result(std::future<std::shared_ptr<SetBool::Response>>(), 0) { }

    /**
     * @brief Declares ports needed by this node.
     * @return PortsList Needed ports.
     */
    static BT::PortsList providedPorts() {
        return {
            UwrtInput("srv_name"),
            UwrtInput("data"),
            UwrtInput("time_limit_secs")
        };
    }

    /**
     * @brief Initializes ROS peripherals such as publishers, subscribers, actions, services, etc.
     * Anything requiring the ROS node handle to construct should be initialized here. Do not do it in the 
     * constructor or you will be very sad
     */
    void rosInit() override { }

    /**
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override {
        srvName = tryGetRequiredInput<std::string>(this, "srv_name", "/some_srv");
        client = rosnode->create_client<SetBool>(srvName);
        
        //wait for client
        if(!client->wait_for_service(1s)) {
            RCLCPP_ERROR(rosNode()->get_logger(), "SetBool Service %s is not available.", srvName.c_str());
            return BT::NodeStatus::FAILURE;
        }

        //assemble request
        auto request = std::make_shared<SetBool::Request>();
        bool data = tryGetRequiredInput<bool>(this, "data", false);
        request->data = data;

        //send and mark send time
        result = client->async_send_request(request);
        startTime = rosnode->get_clock()->now();

        //store timeout
        timeoutSecs = tryGetRequiredInput<double>(this, "time_limit_secs", 1);
        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after 
     */
    BT::NodeStatus onRunning() override {
        //check if std::future stored in result is valid
        if(!result.valid()) {
            RCLCPP_ERROR(rosNode()->get_logger(), "Result of SetBool service call to %s is invalid!", srvName.c_str());
            return BT::NodeStatus::FAILURE;
        }

        //check if result is ready
        if(result.wait_for(0s) == std::future_status::ready) {
            //only do one call to get(); any more crash because of std::move() type stuff
            SetBool::Response::SharedPtr resp = result.get(); 
            std::string message = resp->message;
            bool success = resp->success;
            
            //report the message in the response if there is one
            if(message.length() > 0) {
                RCLCPP_WARN(rosNode()->get_logger(), "Message from %s: %s", srvName.c_str(), message.c_str());
            }

            return (success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE);
        }

        //...okay its not ready. have we timed out yet?
        if((rosnode->get_clock()->now() - startTime).seconds() > timeoutSecs) {
            RCLCPP_ERROR(rosNode()->get_logger(), "Service call to %s took to long to respond.", srvName.c_str());
            return BT::NodeStatus::FAILURE;
        }
        
        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called when the node is halted.
     */
    void onHalted() override {

    }

    private:
    std::string srvName;
    rclcpp::Client<SetBool>::SharedPtr client;
    rclcpp::Client<SetBool>::FutureAndRequestId result;
    rclcpp::Time startTime;
    double timeoutSecs;
};
