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
            UwrtInput("from_frame"),
            UwrtInput("to_frame"),
            UwrtInput("x"),
            UwrtInput("y"),
            UwrtInput("z"),
            UwrtInput("or"),
            UwrtInput("op"),
            UwrtInput("oy"),
            UwrtOutput("out_x"),
            UwrtOutput("out_y"),
            UwrtOutput("out_z"),
            UwrtOutput("out_or"),
            UwrtOutput("out_op"),
            UwrtOutput("out_oy")
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
            fromFrame = tryGetRequiredInput<std::string>(this, "from_frame", "world"),
            toFrame = tryGetRequiredInput<std::string>(this, "to_frame", "world");

        geometry_msgs::msg::Pose original;
        original.position.x = tryGetRequiredInput<double>(this, "x", 0);
        original.position.y = tryGetRequiredInput<double>(this, "y", 0);
        original.position.z = tryGetRequiredInput<double>(this, "z", 0);
        
        //convert original rpy to quaternion and set that
        geometry_msgs::msg::Vector3 originalRPY;
        originalRPY.x = tryGetRequiredInput<double>(this, "or", 0);
        originalRPY.y = tryGetRequiredInput<double>(this, "op", 0);
        originalRPY.z = tryGetRequiredInput<double>(this, "oy", 0);
        original.orientation = toQuat(originalRPY);
        

        geometry_msgs::msg::Pose transformed;
        if(transformBetweenFrames(rosnode, buffer, original, fromFrame, toFrame, transformed)){
            geometry_msgs::msg::Vector3 transformedRPY = toRPY(transformed.orientation);

            //set output ports
            postOutput<double>(this,"out_x", transformed.position.x);
            postOutput<double>(this,"out_y", transformed.position.y);
            postOutput<double>(this,"out_z", transformed.position.z);

            //convert orientation back to RPY and return that
            postOutput<double>(this, "out_or", transformedRPY.x);
            postOutput<double>(this, "out_op", transformedRPY.y);
            postOutput<double>(this, "out_oy", transformedRPY.z);
            
            return BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_ERROR(rosNode()->get_logger(), "Failed to look up transform from %s to %s!", fromFrame.c_str(), toFrame.c_str());
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
