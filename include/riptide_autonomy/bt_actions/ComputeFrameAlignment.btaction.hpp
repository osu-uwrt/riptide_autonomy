#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

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
            UwrtInput("x"),
            UwrtInput("y"),
            UwrtInput("z"),
            UwrtInput("frameName"),
            UwrtOutput("out_x"),
            UwrtOutput("out_y"),
            UwrtOutput("out_z"),
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
        std::string 
            ns = rosnode->get_namespace(),
            robotName = ns.substr(1, ns.find('/', 1)), // start at 1 to skip leading /
            finalFrame = tryGetRequiredInput<std::string>(this, "frameName", ""),
            fromOdom = "world",
            BaseFrame = robotName + "/base_link";

        //make a pose  that represents the desired position of the given frame
        geometry_msgs::msg::Pose finalRes;
        finalRes.position.x = tryGetRequiredInput<double>(this, "x", 0);
        finalRes.position.y = tryGetRequiredInput<double>(this, "y", 0);
        finalRes.position.z = tryGetRequiredInput<double>(this, "z", 0);

        geometry_msgs::msg::Pose baseFrameOdom;
        if(!transformBetweenFrames(rosnode, geometry_msgs::msg::Pose(), BaseFrame, fromOdom, baseFrameOdom)) {
            return BT::NodeStatus::FAILURE;
        }

        geometry_msgs::msg::Pose targetFrameOdom;
        if(!transformBetweenFrames(rosnode, geometry_msgs::msg::Pose(), finalFrame, fromOdom, targetFrameOdom)) {
            return BT::NodeStatus::FAILURE;
        }

        //compute target tempest position by adding target frame displacement in odom frame to the target position
        finalRes.position.x -= targetFrameOdom.position.x - baseFrameOdom.position.x;
        finalRes.position.y -= targetFrameOdom.position.y - baseFrameOdom.position.y;
        finalRes.position.z -= targetFrameOdom.position.z - baseFrameOdom.position.z;

        //set outputs to the odom coords that get the desired frame to the desired position

        postOutput<double>(this, "out_x", finalRes.position.x);
        postOutput<double>(this, "out_y", finalRes.position.y);
        postOutput<double>(this, "out_z", finalRes.position.z);

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
