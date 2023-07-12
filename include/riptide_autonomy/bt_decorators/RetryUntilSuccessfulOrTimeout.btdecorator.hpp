#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

class RetryUntilSuccessfulOrTimeout : public UWRTDecoratorNode {
    public:
    RetryUntilSuccessfulOrTimeout(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTDecoratorNode(name, config) {
        // nodeStarted = false;
        duration = 0;
    }

    /**
     * @brief Declares ports needed by this node.
     * @return PortsList Needed ports.
     */
    static BT::PortsList providedPorts() {
        return {
            UwrtInput("num_seconds")
        };
    }

    /**
     * @brief Initializes ROS peripherals such as publishers, subscribers, actions, services, etc.
     * Anything requiring the ROS node handle to construct should be initialized here. Do not do it in the 
     * constructor or you will be very sad
     */
    void rosInit() override { 

    }

    /**
     * @brief Executes the node.
     * This method will be called once by the tree and can block for as long
     * as it needs for the action to be completed. When execution completes,
     * this method must return either SUCCESS or FAILURE; it CANNOT return
     * IDLE or RUNNING.
     *
     * @return NodeStatus The result of the execution; SUCCESS or FAILURE.
     */
    BT::NodeStatus tick() override {
        // if(!nodeStarted) { //first time running node
        if(status() == BT::NodeStatus::IDLE) {
            startTime = rosnode->get_clock()->now();
            duration = tryGetRequiredInput<double>(this, "num_seconds", 0);
            // nodeStarted = true;
        }
        
        double timeElapsed = (rosnode->get_clock()->now() - startTime).seconds();
        if(timeElapsed < duration) {
            //have not run duration yet. either succeed or retry
            BT::NodeStatus result = child()->executeTick();
            if(result == BT::NodeStatus::SUCCESS) {
                // nodeStarted = false; //node no longer running
                return BT::NodeStatus::SUCCESS;
            }

            return BT::NodeStatus::RUNNING; //we will get ticked again
        } 
        
        //timeElapsed > duration
        RCLCPP_ERROR(log, "RetryUntilSuccessfulOrTimeout named \"%s\" timed out.", this->name().c_str());
        return BT::NodeStatus::FAILURE;
    }

    private:
    // bool nodeStarted;
    double duration;
    rclcpp::Time startTime;
};

