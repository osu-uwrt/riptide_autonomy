#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

using Cov = geometry_msgs::msg::PoseWithCovarianceStamped;
using namespace std::placeholders;
using namespace std::chrono_literals;


class getCovariance : public UWRTActionNode {
    public:
    getCovariance(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config) {
        
    }

    /**
     * @brief Declares ports needed by this node.
     * @return PortsList Needed ports.
     */
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("Target"),
            BT::OutputPort<double>("Covariance")
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
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override {
        topicName = "mapping/" + tryGetRequiredInput<std::string>(this, "Target", "ERROR_VALUE");
        subscriber = rosnode->create_subscription<Cov>(topicName,  rclcpp::SensorDataQoS(), std::bind(&getCovariance::topic_callback, this, _1));
        
        msgReceived = false;
        startTime = rosnode->get_clock()->now();
        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after 
     */
    BT::NodeStatus onRunning() override {
        if(msgReceived) {
            setOutput<double>("Covariance", error);
            return BT::NodeStatus::SUCCESS;
        }

        if(rosnode->get_clock()->now() - startTime > 5s) {
            RCLCPP_ERROR(log, "Timed out waiting on topic %s", topicName.c_str());
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
    void topic_callback(const Cov::SharedPtr msg)
    {
        error = 0;

        for(int i = 0; i < 6; i++){
            error += msg.get()->pose.covariance.at((6*i) + i);
        }

        msgReceived = true;
    }

    bool msgReceived;
    rclcpp::Subscription<Cov>::SharedPtr subscriber;
    double error;
    rclcpp::Time startTime;
    std::string topicName;
};
