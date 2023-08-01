#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"


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
            UwrtInput("x"),
            UwrtInput("y"),
            UwrtInput("z"),
            UwrtInput("or"),
            UwrtInput("op"),
            UwrtInput("oy"),
            UwrtInput("reference_frame"), //the frame to align relative to
            UwrtInput("target_frame"),    //the frame to align
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
        std::string
            ns = rosnode->get_namespace(),
            robotName = ns.substr(1, ns.find('/', 1));
        
        baseLinkName = robotName + "/base_link";
    }

    /**
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override {
        poseIn.position.x = tryGetRequiredInput<double>(this, "x", 0);
        poseIn.position.y = tryGetRequiredInput<double>(this, "y", 0);
        poseIn.position.z = tryGetRequiredInput<double>(this, "z", 0);

        geometry_msgs::msg::Vector3 rpy;
        rpy.x = tryGetRequiredInput<double>(this, "or", 0);
        rpy.y = tryGetRequiredInput<double>(this, "op", 0);
        rpy.z = tryGetRequiredInput<double>(this, "oy", 0);

        poseIn.orientation  = toQuat(rpy);
        referenceFrameName  = tryGetRequiredInput<std::string>(this, "reference_frame", "");
        targetFrameName     = tryGetRequiredInput<std::string>(this, "target_frame", "");
        startTime           = rosnode->get_clock()->now();

        haveReferenceToWorld = false;
        haveBaselinkToTarget = false;
        haveTargetFrametoWorld = false;

        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after 
     */
    BT::NodeStatus onRunning() override {
        // here, look up TWO transforms:
        // - transform from base link to the target frame
        // - transform from reference link to world
        //
        // after the transforms are looked up:
        // - apply the base link to target frame transform to poseIn
        // - apply reference link to world transform to the result of the previous application.

        if(!haveReferenceToWorld) {
            haveReferenceToWorld = lookupTransformThrottled(rosNode(), tfBuffer, referenceFrameName, "world", 0.5, referenceToWorldTimer, referenceToWorldTransform);
        }

        if(!haveBaselinkToTarget) {
            haveBaselinkToTarget = lookupTransformThrottled(rosNode(), tfBuffer, baseLinkName, targetFrameName, 0.5, baseLinkToTargetTimer, baselinkToTargetTransform);
        }

        if(!haveTargetFrametoWorld) {
            haveTargetFrametoWorld = lookupTransformThrottled(rosNode(), tfBuffer, targetFrameName, "world", 0.5, targetFrameToWorldTimer, targetFrameToWorldTransform);
        }

        if(haveReferenceToWorld && haveBaselinkToTarget) {
            //rotate the translation vector so that the pose is transformed relative to its rotation
            // tf2::Quaternion inRotationTf;
            // tf2::Vector3 translationTf;
            // tf2::fromMsg(poseIn.orientation, inRotationTf);
            // tf2::fromMsg(baselinkToTargetTransform.transform.translation, translationTf);
            // tf2::Vector3 rotatedTranslation = tf2::quatRotate(inRotationTf, translationTf);
            // baselinkToTargetTransform.transform.translation = tf2::toMsg(rotatedTranslation);
            
            geometry_msgs::msg::Pose 
                goalPose = doTransform(poseIn, referenceToWorldTransform),
                targetFrameBaselinkPose = doTransform(geometry_msgs::msg::Pose(), baselinkToTargetTransform),
                targetDisplacementWorld = doTransform(targetFrameBaselinkPose, targetFrameToWorldTransform),
                outputPose;


            // geometry_msgs::msg::Pose 
            //     targetFramePose = doTransform(poseIn, referenceToWorldTransform),
            //     outputPose      = doTransform(targetFramePose, baselinkToTargetTransform);
            
            // printTransform(rosNode(), "base link to transform", baselinkToTargetTransform);
            // printTransform(rosNode(), )
            
            postOutput<double>(this, "out_x", outputPose.position.x);
            postOutput<double>(this, "out_y", outputPose.position.y);
            postOutput<double>(this, "out_z", outputPose.position.z);
            
            geometry_msgs::msg::Vector3 outRpy = toRPY(outputPose.orientation);
            postOutput<double>(this, "out_or", outRpy.x);
            postOutput<double>(this, "out_op", outRpy.y);
            postOutput<double>(this, "out_oy", outRpy.z);
            return BT::NodeStatus::SUCCESS;
        }

        return (rosnode->get_clock()->now() - startTime < 3s ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE);
    }

    /**
     * @brief Called when the node is halted.
     */
    void onHalted() override {

    }

    private:
    DEF_THROTTLE_TIMER(referenceToWorldTimer);
    DEF_THROTTLE_TIMER(baseLinkToTargetTimer);
    DEF_THROTTLE_TIMER(targetFrameToWorldTimer);

    geometry_msgs::msg::Pose poseIn;
    rclcpp::Time startTime;
    std::string 
        referenceFrameName,
        targetFrameName,
        baseLinkName;
    bool
        haveReferenceToWorld,
        haveBaselinkToTarget,
        haveTargetFrametoWorld;
    geometry_msgs::msg::TransformStamped
        referenceToWorldTransform,
        baselinkToTargetTransform,
        targetFrameToWorldTransform;
};
