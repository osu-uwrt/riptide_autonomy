#include "autonomy.h"

using namespace BT;

/**
 * @brief State that converts relative coordinates to global (world frame) ones.
 * 
 * This state receives a frame/object name, a set of coordinates relative to that 
 * frame/object, and an orientation relative to that frame/object, and transforms 
 * those coordinates to global or "world frame" coordinates. This state can be used,
 * for example, if you want to move the robot 1 meter directly in front of the gate.
 * Use gate_frame as the frame name, the coordinates 1, 0, 0, and the orientation 
 * 0, 0, -1, 1, and then use the move state with the output values from this state.
 */
class ToWorldFrameState : public UWRTSyncActionNode {
    public:

    ToWorldFrameState(const std::string& name, const NodeConfiguration& config)
     : UWRTSyncActionNode(name, config) { }

    /**
     * @brief Initializes the node. 
     * 
     * @param node The ROS node belonging to the current process.
     */
    void init(rclcpp::Node::SharedPtr node) {
        rosnode = node;
    }

    /**
     * @brief Declares ports needed by this state.
     * 
     * @return PortsList Needed ports.
     */
    static PortsList providedPorts() {
        return {
            InputPort<std::string>("object"),
            InputPort<std::string>("relative_x"),
            InputPort<std::string>("relative_y"),
            InputPort<std::string>("relative_z"),
            InputPort<std::string>("relative_orientation_x"),
            InputPort<std::string>("relative_orientation_y"),
            InputPort<std::string>("relative_orientation_z"),
            InputPort<std::string>("relative_orientation_w"),
            OutputPort<std::string>("world_x"),
            OutputPort<std::string>("world_y"),
            OutputPort<std::string>("world_z"),
            OutputPort<std::string>("world_orientation_x"),
            OutputPort<std::string>("world_orientation_y"),
            OutputPort<std::string>("world_orientation_z"),
            OutputPort<std::string>("world_orientation_w")
        };
    }

    /**
     * @brief Executes the node.
     * This method will be called once by the tree and can block for as long
     * as it needs for the action to be completed. When execution completes,
     * this method must return either SUCCESS or FAILURE; it CANNOT return 
     * IDLE or RUNNING.
     * 
     * @return NodeStatus The result of the execution; SUCCESS or FAILURE.
     */
    NodeStatus tick() override {
        //start buffer client to look up transform
    RCLCPP_INFO(log, "Starting TF2 Buffer Server");
    tf2_ros::BufferClient buffer(rosnode, "tf2_buffer_server");
    buffer.waitForServer();

    RCLCPP_INFO(log, "TF2 Buffer Server connected.");

    //look up object (should be broadcasted from mapping)
    std::string objectName = getInput<std::string>("object").value();
    geometry_msgs::msg::TransformStamped transform = buffer.lookupTransform("world", objectName, tf2::TimePointZero, tf2::durationFromSec(1.0));

    geometry_msgs::msg::Pose currentPose;
    geometry_msgs::msg::Pose desiredPose;
    
    //get current pose to translate
    currentPose.position.x = std::stod(getInput<std::string>("relative_x").value());
    currentPose.position.y = std::stod(getInput<std::string>("relative_y").value());
    currentPose.position.z = std::stod(getInput<std::string>("relative_z").value());
    currentPose.orientation.x = std::stod(getInput<std::string>("relative_orientation_x").value());
    currentPose.orientation.y = std::stod(getInput<std::string>("relative_orientation_y").value());
    currentPose.orientation.z = std::stod(getInput<std::string>("relative_orientation_z").value());
    currentPose.orientation.w = std::stod(getInput<std::string>("relative_orientation_w").value());

    //transform the current pose into desired
    tf2::doTransform(currentPose, desiredPose, transform);

    //output desired
    setOutput<std::string>("world_x", std::to_string(desiredPose.position.x));
    setOutput<std::string>("world_y", std::to_string(desiredPose.position.y));
    setOutput<std::string>("world_z", std::to_string(desiredPose.position.z));
    setOutput<std::string>("world_orientation_x", std::to_string(desiredPose.orientation.x));
    setOutput<std::string>("world_orientation_y", std::to_string(desiredPose.orientation.y));
    setOutput<std::string>("world_orientation_z", std::to_string(desiredPose.orientation.z));
    setOutput<std::string>("world_orientation_w", std::to_string(desiredPose.orientation.w));

    RCLCPP_INFO(log, "World Frame Calculated.");

    return NodeStatus::SUCCESS;
    }

    private:
    rclcpp::Node::SharedPtr rosnode;
};