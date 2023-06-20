#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

class ApproxEqualTo : public UWRTConditionNode {
    public:
    ApproxEqualTo(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTConditionNode(name, config) {
        
    }

    /**
     * @brief Declares ports needed by this node.
     * @return PortsList Needed ports.
     */
    static BT::PortsList providedPorts() {
        return {
            UwrtInput("a"),
            UwrtInput("b"),
            UwrtInput("range")
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
        double
        b = tryGetRequiredInput<double>(this, "b", 0),
        a = tryGetRequiredInput<double>(this, "a", 0),
        range = tryGetRequiredInput<double>(this, "range", 0);

        return (abs(a - b) < range ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE);
    }
};

