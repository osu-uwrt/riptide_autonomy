#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

class SetControllerSafeMode : public UWRTActionNode {
    public:
    SetControllerSafeMode(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config) {
        
    }

    /**
     * @brief Declares ports needed by this node.
     * @return PortsList Needed ports.
     */
    static BT::PortsList providedPorts() {
        return {
            UwrtInput("safe"),
            UwrtInput("timeout_secs")
        };
    }

    /**
     * @brief Initializes ROS peripherals such as publishers, subscribers, actions, services, etc.
     * Anything requiring the ROS node handle to construct should be initialized here. Do not do it in the 
     * constructor or you will be very sad
     */
    void rosInit() override { 
        asyncclient = std::make_shared<rclcpp::AsyncParametersClient>(rosnode, "complete_controller");
    }

    /**
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override {
        bool safe = tryGetRequiredInput<bool>(this, "safe", true);
        timeout = tryGetRequiredInput<double>(this, "timeout_secs", 0);
        setcomplete = false;
        std::vector<rclcpp::Parameter> params;
        rclcpp::ParameterValue paramvalue(safe);
        rclcpp::Parameter safeparam("controller__stunt__safe_mode", paramvalue);
        params.push_back(safeparam);
        asyncclient->set_parameters(params, std::bind(&SetControllerSafeMode::setParamDoneCb, this, _1));
        starttime = rosnode->get_clock()->now();
        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after 
     */
    BT::NodeStatus onRunning() override {
        if(setcomplete) {
            return BT::NodeStatus::SUCCESS;
        }

        //not complete, check if timed out
        if((rosnode->get_clock()->now() - starttime).seconds() > timeout) {
            RCLCPP_ERROR(rosnode->get_logger(), "Attempt to set safe mode timed out.");
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
    void setParamDoneCb(std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
        std::vector<rcl_interfaces::msg::SetParametersResult> results = future.get();
        
        //only set 1 parameter so should only check that
        if(results.size() != 1) {
            RCLCPP_ERROR(rosnode->get_logger(), "Received a result from the set parameters client with an incorrect size");
        }

        rcl_interfaces::msg::SetParametersResult result = results[0];
        if(result.successful) {
            setcomplete = true;
        } else {
            RCLCPP_ERROR(rosnode->get_logger(), "Failed to set parameter for reason: %s", result.reason.c_str());
        }
    }

    bool setcomplete;
    double timeout;
    rclcpp::Time starttime;
    rclcpp::AsyncParametersClient::SharedPtr asyncclient;
};
