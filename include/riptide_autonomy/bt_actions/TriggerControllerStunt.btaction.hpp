#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

#define STUNT_STATE_TRIGGER_TOPIC "/talos/controller/stunt_state"
#define STUNT_STATE_STATUS_TOPIC "/talos/controller/running_stunt_state"
#define MAX_RETRY 5
#define RETRY_INTERVAL 2

class TriggerControllerStunt : public UWRTActionNode {
    public:
    TriggerControllerStunt(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config) {
        
    }

    /**
     * @brief Declares ports needed by this node.
     * @return PortsList Needed ports.
     */
    static BT::PortsList providedPorts() {
        return {
            UwrtInput("targetStuntState")
        };
    }

    /**
     * @brief Initializes ROS peripherals such as publishers, subscribers, actions, services, etc.
     * Anything requiring the ROS node handle to construct should be initialized here. Do not do it in the 
     * constructor or you will be very sad
     */
    void rosInit() override { 
        //get the name of the complete controller
        // std::vector<std::string> nodeNames = rosnode->get_node_names();
        // std::string controllerNodeName = "";

        // for(size_t i = 0; i < nodeNames.size(); i++){

        //     std::string controllerSeedString = "/complete_controller";
        //     if(nodeNames.at(i).substr(0,7) == controllerSeedString.substr(0,7)){
        //         RCLCPP_INFO(rosnode->get_logger(), "Found Controller to disable safe mode lol");
        //         controllerNodeName = nodeNames.at(i);
        //     }
        // }

        stuntStateSub = rosnode->create_subscription<std_msgs::msg::UInt16>(STUNT_STATE_STATUS_TOPIC, 10, std::bind(&TriggerControllerStunt::running_stunt_state_cb, this, _1));
        stuntStatePub = rosnode->create_publisher<std_msgs::msg::UInt16>(STUNT_STATE_TRIGGER_TOPIC, 10);


    }

    /**
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override {

        startTime = rosnode->get_clock()->now();

        std_msgs::msg::UInt16 msg;
        msg.data = tryGetRequiredInput<int>(this, "targetStuntState", 0);
        stuntStatePub->publish(msg);

        return BT::NodeStatus::SUCCESS;
    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after 
     */
    BT::NodeStatus onRunning() override {
        if(stunt_state == tryGetRequiredInput<int>(this, "targetStuntState", 0)){
            return BT::NodeStatus::SUCCESS;
        }

        if(rosnode->get_clock()->now().seconds() > startTime.seconds() + retry_count + RETRY_INTERVAL){
            std_msgs::msg::UInt16 msg;
            msg.data = tryGetRequiredInput<int>(this, "targetStuntState", 0);
            stuntStatePub->publish(msg);

            retry_count++;
        }

        if(retry_count > MAX_RETRY){
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

    void running_stunt_state_cb(const std_msgs::msg::UInt16::SharedPtr msg) {
        stunt_state = msg->data;
    }

    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr stuntStateSub;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr stuntStatePub;

    uint16_t stunt_state = 0;
    rclcpp::Time startTime;
    uint8_t retry_count = 0;
};
