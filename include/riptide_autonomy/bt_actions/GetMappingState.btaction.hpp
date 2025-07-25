#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

class GetMappingState : public UWRTActionNode {
    public:
    GetMappingState(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config) {
        
    }

    /**
     * @brief Declares ports needed by this node.
     * @return PortsList Needed ports.
     */
    static BT::PortsList providedPorts() {
        return {
            UwrtOutput("map_locked"),
            UwrtOutput("target_name")
        };
    }

    /**
     * @brief Initializes ROS peripherals such as publishers, subscribers, actions, services, etc.
     * Anything requiring the ROS node handle to construct should be initialized here. Do not do it in the 
     * constructor or you will be very sad
     */
    void rosInit() override { 
        _sub = rosNode()->create_subscription<riptide_msgs2::msg::MappingTargetInfo>(MAPPING_TARGET_INFO_TOPIC, 10, std::bind(&GetMappingState::infoCb, this, _1));
    }

    /**
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override {
        _hasMsg = false;
        _startTime = rosNode()->get_clock()->now();
        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after 
     */
    BT::NodeStatus onRunning() override {
        rclcpp::Time now = rosNode()->get_clock()->now();

        if(_hasMsg)
        {
            postOutput<bool>(this, "map_locked", _latestMsg.lock_map);
            postOutput<std::string>(this, "target_name", _latestMsg.target_object);
            return BT::NodeStatus::SUCCESS;
        }

        if(now - _startTime > 3s)
        {
            RCLCPP_ERROR(rosNode()->get_logger(), "Timed out waiting for mapping state.");
            postOutput<bool>(this, "map_locked", false);
            postOutput<std::string>(this, "target_name", "");
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
    void infoCb(const riptide_msgs2::msg::MappingTargetInfo::SharedPtr msg)
    {
        _latestMsg = *msg;
        _hasMsg = true;
    }

    bool _hasMsg;
    rclcpp::Time _startTime;
    riptide_msgs2::msg::MappingTargetInfo _latestMsg;
    rclcpp::Subscription<riptide_msgs2::msg::MappingTargetInfo>::SharedPtr _sub;
};
