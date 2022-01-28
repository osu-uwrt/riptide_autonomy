#include "autonomy.h"

using namespace BT;
using namespace std::placeholders;

/**
 * @brief Line-drive state
 * 
 * This state calculates the position that the robot should move to in order 
 * to drive directly through a target frame.
 */
class LineDriveCalcState : public UWRTSyncActionNode {
    public:

    LineDriveCalcState(const std::string& name, const NodeConfiguration& config)
     : UWRTSyncActionNode(name, config) { }

    /**
     * @brief Initializes the node. 
     * 
     * @param node The ROS node belonging to the current process.
     */
    void init(rclcpp::Node::SharedPtr node) {
        rosnode = node;
        odomSub = node->create_subscription<nav_msgs::msg::Odometry>(ODOMETRY_TOPIC, 10, std::bind(&LineDriveCalcState::odomCallback, this, _1));
    }

    /**
     * @brief Declares ports needed by this state.
     * 
     * @return PortsList Needed ports.
     */
    static PortsList providedPorts() {
        return {
            InputPort("frame"),
            InputPort("extra_distance"),
            OutputPort("x_out"),
            OutputPort("y_out"),
            OutputPort("z_out")
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
        //wait for localization data
        RCLCPP_INFO(log, "Waiting for loc data");

        while(!odomExists) {
            rclcpp::spin_some(rosnode);
        }

        //get frame from TF
        RCLCPP_INFO(log, "Starting TF2 Buffer Server");
        tf2_ros::BufferClient buffer(rosnode, "tf2_buffer_server");
        buffer.waitForServer();

        RCLCPP_INFO(log, "TF2 Buffer Server connected.");

        //look up object (should be broadcasted from mapping)
        std::string objectName = getInput<std::string>("frame").value();
        geometry_msgs::msg::TransformStamped transform = buffer.lookupTransform("world", objectName, tf2::TimePointZero, tf2::durationFromSec(1.0));

        geometry_msgs::msg::Pose relativePose; //we don't want a relative pose, we want the exact thing
        geometry_msgs::msg::Pose objectPose; // will be the exact pose of the object
        tf2::doTransform(relativePose, objectPose, transform);

        double
            deltaY     = objectPose.position.y - currentPose.position.y,
            deltaX     = objectPose.position.x - currentPose.position.x,
            distance   = sqrt((deltaY * deltaY) + (deltaX * deltaX)),
            extraDistance = std::stod(getInput<std::string>("extra_distance").value()),
            distFactor = (distance + extraDistance) / distance;

        deltaY *= distFactor;
        deltaX *= distFactor;

        geometry_msgs::msg::Vector3 target;

        target.x = currentPose.position.x + deltaX;
        target.y = currentPose.position.y + deltaY;

        setOutput<std::string>("x_out", std::to_string(target.x));
        setOutput<std::string>("y_out", std::to_string(target.y));
        setOutput<std::string>("z_out", std::to_string(target.z));

        return NodeStatus::SUCCESS;
    }

    private:
    /**
     * @brief Called when the odometry subscription receives a message.
     * 
     * @param msg The received message.
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        currentPose = msg->pose.pose;
        odomExists = true;
    }

    rclcpp::Node::SharedPtr rosnode;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;
    geometry_msgs::msg::Pose currentPose;

    bool odomExists;
};