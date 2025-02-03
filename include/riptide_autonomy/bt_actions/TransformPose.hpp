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
    void rosInit() override { }

    /**
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override {
        // get frame names
        fromFrame = tryGetRequiredInput<std::string>("from_frame", "world");
        toFrame = tryGetRequiredInput<std::string>("to_frame", "world");

        original.position.x = tryGetRequiredInput<double>("x", 0);
        original.position.y = tryGetRequiredInput<double>("y", 0);
        original.position.z = tryGetRequiredInput<double>("z", 0);
        
        //convert original rpy to quaternion and set that
        geometry_msgs::msg::Vector3 originalRPY;
        originalRPY.x = tryGetRequiredInput<double>("or", 0);
        originalRPY.y = tryGetRequiredInput<double>("op", 0);
        originalRPY.z = tryGetRequiredInput<double>("oy", 0);
        original.orientation = toQuat(originalRPY);

        startTime = rosNode()->get_clock()->now();
        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after 
     */
    BT::NodeStatus onRunning() override {
        std::string 
            from = tryGetRequiredInput<std::string>("from_frame", ""),
            to = tryGetRequiredInput<std::string>("to_frame", "");

        bool res = lookupTransformThrottled(rosNode(), tfBuffer, from, to, 0.5, lookupTimer, transform);
        if(res) {
            //lookup success! apply transform and set outputs
            geometry_msgs::msg::Pose result = doTransform(original, transform);
            
            postOutput<double>("out_x", result.position.x);
            postOutput<double>("out_y", result.position.y);
            postOutput<double>("out_z", result.position.z);

            geometry_msgs::msg::Vector3 outRPY = toRPY(result.orientation);
            postOutput<double>("out_or", outRPY.x);
            postOutput<double>("out_op", outRPY.y);
            postOutput<double>("out_oy", outRPY.z);

            RCLCPP_DEBUG(rosNode()->get_logger(), "Transform from %s to %s looked up as XYZ %.3f, %.3f, %.3f and RPY %.3f, %.3f, %.3f",
                from.c_str(),
                to.c_str(),
                result.position.x,
                result.position.y,
                result.position.z,
                outRPY.x,
                outRPY.y,
                outRPY.z);

            return BT::NodeStatus::SUCCESS;
        }

        //if we get down here, lookup has not been completed yet
        return (rosNode()->get_clock()->now() - startTime < 3s ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE);
    }

    /**
     * @brief Called when the node is halted.
     */
    void onHalted() override {

    }

    private:
    DEF_THROTTLE_TIMER(lookupTimer);
    std::string 
        fromFrame,
        toFrame;
    
    geometry_msgs::msg::Pose original;
    geometry_msgs::msg::TransformStamped transform;
    rclcpp::Time startTime;
};
