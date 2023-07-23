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
            UwrtInput("or"),
            UwrtInput("op"),
            UwrtInput("oy"),
            UwrtInput("link_name"),
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

        poseIn.orientation = toQuat(rpy);
        linkName = tryGetRequiredInput<std::string>(this, "link_name", "");
        startTime = rosnode->get_clock()->now();
        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after 
     */
    BT::NodeStatus onRunning() override {
        //here, look up transform between base_link and the link to align. Apply 
        //that transform to the inPose to get the output pose
        if(lookupTransformThrottled(rosnode, tfBuffer, baseLinkName, linkName, 0.5, lookupTimer, transform)) {
            //rotate the translation vector so that the pose is transformed relative to its rotation
            tf2::Quaternion inRotationTf;
            tf2::Vector3 translationTf;
            tf2::fromMsg(poseIn.orientation, inRotationTf);
            tf2::fromMsg(transform.transform.translation, translationTf);
            tf2::Vector3 rotatedTranslation = tf2::quatRotate(inRotationTf, translationTf);
            transform.transform.translation = tf2::toMsg(rotatedTranslation);

            geometry_msgs::msg::Pose outPose = doTransform(poseIn, transform);

            postOutput<double>(this, "out_x", outPose.position.x);
            postOutput<double>(this, "out_y", outPose.position.y);
            postOutput<double>(this, "out_z", outPose.position.z);

            geometry_msgs::msg::Vector3 outRpy = toRPY(outPose.orientation);
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
    DEF_THROTTLE_TIMER(lookupTimer);
    geometry_msgs::msg::Pose poseIn;
    geometry_msgs::msg::TransformStamped transform;
    rclcpp::Time startTime;
    std::string 
        linkName,
        baseLinkName;
};
