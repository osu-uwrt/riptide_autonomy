#pragma once

#include "riptide_autonomy/autonomy_base.hpp"
#include "riptide_autonomy/uwrt_node_types.hpp"

#include <tf2_ros/static_transform_broadcaster.h>

inline void printTransform(rclcpp::Node::SharedPtr node, const std::string& msg, const geometry_msgs::msg::TransformStamped& transform) {
    geometry_msgs::msg::Vector3 rotationRpy = toRPY(transform.transform.rotation);
    RCLCPP_INFO(node->get_logger(), 
        "%s: %f %f %f %f %f %f",
        msg.c_str(),
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z,
        rotationRpy.x,
        rotationRpy.y,
        rotationRpy.z);
}

inline void printPose(rclcpp::Node::SharedPtr node, const std::string& msg, const geometry_msgs::msg::Pose& pose) {
    geometry_msgs::msg::Vector3 rotationRpy = toRPY(pose.orientation);
    RCLCPP_INFO(node->get_logger(),
        "%s: %f %f %f %f %f %f",
        msg.c_str(),
        pose.position.x,
        pose.position.y,
        pose.position.z,
        rotationRpy.x,
        rotationRpy.y,
        rotationRpy.z);
}

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
            UwrtInput("x", PORT_REQUIRED),
            UwrtInput("y", PORT_REQUIRED),
            UwrtInput("z", PORT_REQUIRED),
            
            UwrtInput("or", PORT_REQUIRED,
                "roll"),
            
            UwrtInput("op", PORT_REQUIRED,
                "pitch"),
            
            UwrtInput("oy", PORT_REQUIRED,
                "yaw"),
            
            UwrtInput("reference_frame", PORT_REQUIRED,
                "The frame in which the input pose resides"), //the frame to align relative to
            
            UwrtInput("target_frame", PORT_REQUIRED,
                "The robot link to align"),    //the frame to align
            
            UwrtOutput("out_x",
                "world-frame coordinate for robot"),
            
            UwrtOutput("out_y",
                "world-frame coordinate for robot"),
            
            UwrtOutput("out_z",
                "world-frame coordinate for robot"),
            
            UwrtOutput("out_or",
                "world-frame coordinate for robot"),
            
            UwrtOutput("out_op",
                "world-frame coordinate for robot"),
            
            UwrtOutput("out_oy",
                "world-frame coordinate for robot")
        };
    }

    /**
     * @brief Initializes ROS peripherals such as publishers, subscribers, actions, services, etc.
     * Anything requiring the ROS node handle to construct should be initialized here. Do not do it in the 
     * constructor or you will be very sad
     */
    void rosInit() override {
        std::string
            ns = rosNode()->get_namespace(),
            robotName = ns.substr(1, ns.find('/', 1));
        
        baseLinkName = robotName + "/base_link";
        goalPoseFrameName = robotName + "/computeframealign_" + std::to_string(UID()) + "_goalpose";
        goalPoseBroadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(rosNode());
    }

    /**
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override {
        //configure member vars
        referenceFrameName      = tryGetRequiredInput<std::string>("reference_frame", "");
        targetFrameName         = tryGetRequiredInput<std::string>("target_frame", "");
        startTime               = rosNode()->get_clock()->now();
        haveBaselinkToTarget    = false;
        haveGoalPoseFrameToWorld = false;

        //configure goal transform
        geometry_msgs::msg::TransformStamped goalTransform;
        goalTransform.transform.translation.x = tryGetRequiredInput<double>("x", 0);
        goalTransform.transform.translation.y = tryGetRequiredInput<double>("y", 0);
        goalTransform.transform.translation.z = tryGetRequiredInput<double>("z", 0);

        geometry_msgs::msg::Vector3 rpy;
        rpy.x = tryGetRequiredInput<double>("or", 0);
        rpy.y = tryGetRequiredInput<double>("op", 0);
        rpy.z = tryGetRequiredInput<double>("oy", 0);

        goalTransform.transform.rotation    = toQuat(rpy);
        goalTransform.header.frame_id       = referenceFrameName;
        goalTransform.child_frame_id        = goalPoseFrameName;
        goalTransform.header.stamp          = rosNode()->get_clock()->now();

        goalPoseToWorldTransform.transform.translation.x = 0.123456; //TODO: REMOVE

        //configure static broadcaster
        std::vector<geometry_msgs::msg::TransformStamped> transforms = {
            goalTransform
        };

        goalPoseBroadcaster->sendTransform(transforms);
        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after 
     */
    BT::NodeStatus onRunning() override {
        if(rosNode()->get_clock()->now() - startTime < 500ms) {
            return BT::NodeStatus::RUNNING;
        }

        // here, look up TWO transforms:
        // - transform from base link to the target frame
        // - transform from reference link to world
        //
        // after the transforms are looked up:
        // - apply the base link to target frame transform to poseIn
        // - apply reference link to world transform to the result of the previous application.

        if(!haveBaselinkToTarget) {
            haveBaselinkToTarget = lookupTransformThrottled(rosNode(), tfBuffer, baseLinkName, targetFrameName, 0.5, baselinkToTargetTimer, baselinkToTargetTransform);
        }

        if(!haveGoalPoseFrameToWorld) {
            haveGoalPoseFrameToWorld = lookupTransformThrottled(rosNode(), tfBuffer, goalPoseFrameName, "world", 0.5, goalPoseToWorldTimer, goalPoseToWorldTransform, true);
        }

        if(haveBaselinkToTarget && haveGoalPoseFrameToWorld) {
            //apply transform
            geometry_msgs::msg::Pose relativeTargetFramePose;
            relativeTargetFramePose.position = vector3ToPoint(baselinkToTargetTransform.transform.translation);
            relativeTargetFramePose.orientation = baselinkToTargetTransform.transform.rotation;

            geometry_msgs::msg::Pose baseLinkPose = doTransform(relativeTargetFramePose, goalPoseToWorldTransform);

            //post results
            postOutput<double>("out_x", baseLinkPose.position.x);
            postOutput<double>("out_y", baseLinkPose.position.y);
            postOutput<double>("out_z", baseLinkPose.position.z);

            geometry_msgs::msg::Vector3 baselinkRpy = toRPY(baseLinkPose.orientation);
            postOutput<double>("out_or", baselinkRpy.x);
            postOutput<double>("out_op", baselinkRpy.y);
            postOutput<double>("out_oy", baselinkRpy.z);

            // goalPoseBroadcaster.reset();
            return BT::NodeStatus::SUCCESS;
        }


        // goalPoseBroadcaster.reset();
        return (rosNode()->get_clock()->now() - startTime > 3s ? BT::NodeStatus::FAILURE : BT::NodeStatus::RUNNING);
    }

    /**
     * @brief Called when the node is halted.
     */
    void onHalted() override {

    }

    private:
    DEF_THROTTLE_TIMER(baselinkToTargetTimer);
    DEF_THROTTLE_TIMER(goalPoseToWorldTimer);

    geometry_msgs::msg::TransformStamped 
        baselinkToTargetTransform,
        goalPoseToWorldTransform;

    std::string
        baseLinkName,
        goalPoseFrameName,
        referenceFrameName,
        targetFrameName;
    
    bool
        haveBaselinkToTarget,
        haveGoalPoseFrameToWorld;
    
    rclcpp::Time startTime;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> goalPoseBroadcaster;
};
