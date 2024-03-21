#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

class SetMappingTarget : public UWRTActionNode {
    public:
    using MappingTarget = riptide_msgs2::srv::MappingTarget;

    SetMappingTarget(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config),
      result(std::future<std::shared_ptr<MappingTarget::Response>>(), 0) {
        
    }

    /**
     * @brief Declares ports needed by this node.
     * @return PortsList Needed ports.
     */
    static BT::PortsList providedPorts() {
        return {
            UwrtInput("target_object"),
            UwrtInput("lock_map"),
            UwrtInput("time_limit_secs")
        };
    }

    /**
     * @brief Initializes ROS peripherals such as publishers, subscribers, actions, services, etc.
     * Anything requiring the ROS node handle to construct should be initialized here. Do not do it in the 
     * constructor or you will be very sad
     */
    void rosInit() override { 
        client = rosNode()->create_client<MappingTarget>(MAPPING_SERVER_NAME);
    }

    /**
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override {
        timeoutSecs = tryGetRequiredInput<double>(this, "time_limit_secs", 0);

        //is client available?
        if(!client->wait_for_service(1s)) {
            RCLCPP_ERROR(rosNode()->get_logger(), "Mapping service %s not available.", MAPPING_SERVER_NAME.c_str());
            return BT::NodeStatus::FAILURE;
        }

        //get node arguments
        std::string target = tryGetRequiredInput<std::string>(this, "target_object", "");
        bool lock = tryGetRequiredInput<bool>(this, "lock_map", false);

        //make request
        auto request = std::make_shared<MappingTarget::Request>();
        request->target_object = target;
        request->lock_map = lock;

        //make call
        result = client->async_send_request(request);
        startTime = rosNode()->get_clock()->now();
        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after 
     */
    BT::NodeStatus onRunning() override {
        if(!result.valid()) {
            RCLCPP_ERROR(rosNode()->get_logger(), "Result of MappingTarget service call to %s is not valid.", MAPPING_SERVER_NAME.c_str());
            return BT::NodeStatus::FAILURE;
        }

        if(result.wait_for(0s) == std::future_status::ready) {
            //no response, if the future completed then the call is done. return success
            return BT::NodeStatus::SUCCESS;
        }

        //not ready, check for timeout
        if((rosNode()->get_clock()->now() - startTime).seconds() > timeoutSecs) {
            RCLCPP_ERROR(rosNode()->get_logger(), "Service call to %s timed out.", MAPPING_SERVER_NAME.c_str());
            return BT::NodeStatus::FAILURE;
        }

        //we'll get em next time
        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called when the node is halted.
     */
    void onHalted() override {

    }

    private:
    double timeoutSecs;
    rclcpp::Time startTime;
    rclcpp::Client<MappingTarget>::SharedPtr client;
    rclcpp::Client<MappingTarget>::FutureAndRequestId result;
};
