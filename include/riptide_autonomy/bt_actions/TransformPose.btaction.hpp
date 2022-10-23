#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

class TransformPose : public UWRTActionNode {
    public:
    TransformPose(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config) {
        
    }

    /**
     * @brief Declares ports needed by this node.
     * @return PortsList Needed ports.
     */
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("from_frame"),
            BT::InputPort<std::string>("to_frame"),
            BT::InputPort<double>("x"),
            BT::InputPort<double>("y"),
            BT::InputPort<double>("z"),
            BT::InputPort<double>("or"),
            BT::InputPort<double>("op"),
            BT::InputPort<double>("oy"),
            BT::OutputPort<double>("out_x"),
            BT::OutputPort<double>("out_y"),
            BT::OutputPort<double>("out_z"),
            BT::OutputPort<double>("out_or"),
            BT::OutputPort<double>("out_op"),
            BT::OutputPort<double>("out_oy")
        };
    }

    /**
     * @brief Initializes ROS peripherals such as publishers, subscribers, actions, services, etc.
     * Anything requiring the ROS node handle to construct should be initialized here. Do not do it in the 
     * constructor or you will be very sad
     */
    void rosInit() override { 
        buffer = std::make_shared<tf2_ros::Buffer>(rosnode->get_clock());
        listener = std::make_shared<tf2_ros::TransformListener>(*buffer);
    }

    /**
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override {
        // get frame names
        std::string
            fromFrame = getInput<std::string>("from_frame").value(),
            toFrame = getInput<std::string>("to_frame").value();

        geometry_msgs::msg::Pose original;
        original.position.x = getInput<double>("x").value();
        original.position.y = getInput<double>("y").value();
        original.position.z = getInput<double>("z").value();
        
        //convert original rpy to quaternion and set that
        geometry_msgs::msg::Vector3 originalRPY;
        originalRPY.x = getInput<double>("or").value();
        originalRPY.y = getInput<double>("op").value();
        originalRPY.z = getInput<double>("oy").value();
        original.orientation = toQuat(originalRPY);
        

        std::tuple<geometry_msgs::msg::Pose, bool> res = transformBetweenFrames(original, fromFrame, toFrame, rosnode, buffer);
        geometry_msgs::msg::Pose transformed = std::get<0>(res);
        geometry_msgs::msg::Vector3 transformedRPY = toRPY(transformed.orientation);

        if(std::get<1>(res)){
            //set output ports
            setOutput<double>("out_x", transformed.position.x);
            setOutput<double>("out_y", transformed.position.y);
            setOutput<double>("out_z", transformed.position.z);

            //convert orientation back to RPY and return that
            RCLCPP_INFO(log, "transformed rpy: %f, %f ,%f", transformedRPY.x, transformedRPY.y, transformedRPY.z);
            setOutput<double>("out_or", transformedRPY.x);
            setOutput<double>("out_op", transformedRPY.y);
            setOutput<double>("out_oy", transformedRPY.z);
            
            return BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_ERROR(log, "Failed to look up transform from %s to %s!", fromFrame.c_str(), toFrame.c_str());
            return BT::NodeStatus::FAILURE;
        }
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

    private:
    std::shared_ptr<tf2_ros::Buffer> buffer;
    std::shared_ptr<tf2_ros::TransformListener> listener;
};
