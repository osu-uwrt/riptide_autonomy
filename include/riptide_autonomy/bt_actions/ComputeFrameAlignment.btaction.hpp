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
            UwrtInput("link_frame"),    //the frame to align
            UwrtInput("base_frame"),    //robot base (e.g. talos/base_link)
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

    }

    /**
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override {
        //get user inputs
        refFrame = tryGetRequiredInput<std::string>(this, "reference_frame", "");
        linkFrame = tryGetRequiredInput<std::string>(this, "link_frame", "");
        baseFrame = tryGetRequiredInput<std::string>(this, "base_frame", "");

        inputPose.transform.translation.x = tryGetRequiredInput<double>(this, "x", 0);
        inputPose.transform.translation.y = tryGetRequiredInput<double>(this, "y", 0);
        inputPose.transform.translation.z = tryGetRequiredInput<double>(this, "z", 0);

        geometry_msgs::msg::Vector3 inRpy;
        inRpy.x = tryGetRequiredInput<double>(this, "or", 0),
        inRpy.y = tryGetRequiredInput<double>(this, "op", 0),
        inRpy.z = tryGetRequiredInput<double>(this, "oy", 0);
        
        inputPose.transform.rotation = toQuat(inRpy);

        std::stringstream msg;
        msg << "Aligning link " << linkFrame << " within frame " << refFrame << " at position";
        printTransform(rosNode(), msg.str(), inputPose);

        //reset state
        haveTtb = false;
        haveTwr = false;

        //start the clock
        startTime = rosNode()->get_clock()->now();
        
        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after 
     */
    BT::NodeStatus onRunning() override {
        // The purpose of this node is to align the target frame, which is rigidly attached to a base frame whose
        // pose is regulated by a controller, to a pose within the reference frame.
        //
        // Need two transforms:
        //  transform from target frame to base frame (call it Ttb)
        //  transform from world to reference frame (Twr)
        //
        // We already have the desired transform from reference frame to target frame: the input pose (call it Trt)
        //
        // With these transforms we can compute the desired transform from world to talos base link (Twb):
        //  Twb = Twr * Trt * Ttb

        //attempt lookups
        if(!haveTtb)
        {
            haveTtb = lookupTransform(baseFrame, linkFrame, geomTtb);
        }

        if(!haveTwr)
        {
            haveTwr = lookupTransform(refFrame, "world", geomTwr);
        }

        // //if we have lookups, perform calculation
        if(haveTtb && haveTwr)
        {
            //convert geometry msgs types to tf types
            tf2::Transform
                tfTtb,
                tfTwr,
                tfTrt;

            tfTtb = geometryMsgsToTf2Transform(geomTtb);
            tfTwr = geometryMsgsToTf2Transform(geomTwr);
            tfTrt = geometryMsgsToTf2Transform(inputPose);
            
            tf2::Transform tfTwb = tfTwr * tfTrt * tfTtb;
            
            //now set outputs
            geometry_msgs::msg::TransformStamped out = tf2TransformToGeometryMsgs(tfTwb);
            postOutput<double>(this, "out_x", out.transform.translation.x);
            postOutput<double>(this, "out_y", out.transform.translation.y);
            postOutput<double>(this, "out_z", out.transform.translation.z);

            geometry_msgs::msg::Vector3 outRpy = toRPY(out.transform.rotation);
            postOutput<double>(this, "out_or", outRpy.x);
            postOutput<double>(this, "out_op", outRpy.y);
            postOutput<double>(this, "out_oy", outRpy.z);

            printTransform(rosNode(), "Alignment computed successfully ", out);
            return BT::NodeStatus::SUCCESS;
        }

        //if we do not have lookups yet (node would have succeeded), check if we have timed out yet
        auto elapsed = rosNode()->get_clock()->now() - startTime;
        if(elapsed < 5s)
        {
            return BT::NodeStatus::RUNNING;
        }

        //too much time has elapsed
        RCLCPP_ERROR(rosNode()->get_logger(), "Failed to calculate alignment");
        return BT::NodeStatus::FAILURE;
    }

    /**
     * @brief Called when the node is halted.
     */
    void onHalted() override {

    }

    private:
    std::string
        refFrame,
        linkFrame,
        baseFrame;

    rclcpp::Time startTime;

    bool
        haveTtb,
        haveTwr;

    geometry_msgs::msg::TransformStamped
        inputPose,
        geomTtb,
        geomTwr;
};
