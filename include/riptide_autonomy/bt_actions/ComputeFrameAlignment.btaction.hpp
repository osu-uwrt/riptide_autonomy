#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

using namespace std::placeholders;

class ComputeFrameAlignment : public UWRTActionNode {
    public:
    ComputeFrameAlignment(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config) { }

    /**
     * @brief Declares ports needed by this node.
     * @return PortsList Needed ports.
     */
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("x"),
            BT::InputPort<double>("y"),
            BT::InputPort<double>("z"),
            BT::InputPort<std::string>("frameName"),
            BT::OutputPort<double>("out_x"),
            BT::OutputPort<double>("out_y"),
            BT::OutputPort<double>("out_z"),
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
        tf2_ros::Buffer buffer(rosnode->get_clock());
        rclcpp::Time startTime = rosnode->get_clock()->now();

        std::string finalFrame = getInput<std::string>("frameName").value();
        std::string fromOdom = "odom";
        std::string BaseFrame = "tempest/base_link";

        //make a pose  that represents the desired position of the given frame
        geometry_msgs::msg::Pose desiredPose;
        desiredPose.position.x = getInput<double>("x").value();
        desiredPose.position.y = getInput<double>("y").value();
        desiredPose.position.z = getInput<double>("z").value();

        //convert this to base frame
        std::tuple<geometry_msgs::msg::Pose, bool> toBase = transformBetweenFrames(desiredPose, BaseFrame, fromOdom, rosnode);
        if(!(std::get<1>(toBase))){
            return BT::NodeStatus::FAILURE;
        }

        //convert base to odom

        desiredPose = std::get<0>(toBase);

        std::tuple<geometry_msgs::msg::Pose, bool> toFinal = transformBetweenFrames(desiredPose, finalFrame, BaseFrame, rosnode);

        if(!(std::get<1>(toFinal))){
            return BT::NodeStatus::FAILURE;
        }

        geometry_msgs::msg::Pose finalRes = std::get<0>(toFinal);

        //set outputs to the odom coords that get the desired frame to the desired position

        setOutput<double>("out_x", finalRes.position.x);
        setOutput<double>("out_y", finalRes.position.y);
        setOutput<double>("out_z", finalRes.position.z);


        return BT::NodeStatus::SUCCESS;
    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after 
     */
    BT::NodeStatus onRunning() override {
        return BT::NodeStatus::SUCCESS;
    }

    /**
     * @brief Called when the node is halted.
     */
    void onHalted() override {

    }
};
