#include "autonomy.h"

using namespace BT;
using namespace std::placeholders;

/**
 * @brief Flatten state.
 * 
 * This state calculates the position and orientation that the robot should move to in 
 * order to properly flatten itself.
 */
class FlattenCalculationState : public UWRTSyncActionNode {
    public:

    FlattenCalculationState(const std::string& name, const NodeConfiguration& config)
     : UWRTSyncActionNode(name, config) { }

    /**
     * @brief Initializes the node. 
     * 
     * @param node The ROS node belonging to the current process.
     */
    void init(rclcpp::Node::SharedPtr node) {
        rosnode = node;
        odomSub = node->create_subscription<nav_msgs::msg::Odometry>(ODOMETRY_TOPIC, 10, std::bind(&FlattenCalculationState::odomCallback, this, _1));

        odomExists = false;
    }

    /**
     * @brief Declares ports needed by this state.
     * 
     * @return PortsList Needed ports.
     */
    static PortsList providedPorts() {
        return {
            InputPort<double>("depth"),
            OutputPort<std::string>("x"),
            OutputPort<std::string>("y"),
            OutputPort<std::string>("z"),
            OutputPort<std::string>("orientation_x"),
            OutputPort<std::string>("orientation_y"),
            OutputPort<std::string>("orientation_z"),
            OutputPort<std::string>("orientation_w")
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
        //wait for loc to exist
        while(!odomExists) {
            rclcpp::spin_some(rosnode);
        }

        geometry_msgs::msg::Pose goalPose;

        //calculate target position (literally the current pose but 1 meter underwater)
        goalPose.position = currentPose.position;
        goalPose.position.z = getInput<double>("depth").value();

        //calculate target orientation (yaw is the same, pitch and roll are 0)
        tf2::Quaternion currentOrientationQuaternion;
        tf2::fromMsg(currentPose.orientation, currentOrientationQuaternion);

        double roll, pitch, yaw;
        tf2::Matrix3x3(currentOrientationQuaternion).getEulerYPR(yaw, pitch, roll);

        //set roll and pitch to 0, "flattening" the robot
        roll = 0;
        pitch = 0;

        //convert rpy back to quaternion
        tf2::Quaternion newOrientationQuaternion;
        newOrientationQuaternion.setRPY(roll, pitch, yaw);
        newOrientationQuaternion.normalize();
        goalPose.orientation.x = newOrientationQuaternion.getX();
        goalPose.orientation.y = newOrientationQuaternion.getY();
        goalPose.orientation.z = newOrientationQuaternion.getZ();
        goalPose.orientation.w = newOrientationQuaternion.getW();

        //set goal pose
        setOutput<std::string>("x", std::to_string(goalPose.position.x));
        setOutput<std::string>("y", std::to_string(goalPose.position.y));
        setOutput<std::string>("z", std::to_string(goalPose.position.z));
        setOutput<std::string>("orientation_x", std::to_string(goalPose.orientation.x));
        setOutput<std::string>("orientation_y", std::to_string(goalPose.orientation.y));
        setOutput<std::string>("orientation_z", std::to_string(goalPose.orientation.z));
        setOutput<std::string>("orientation_w", std::to_string(goalPose.orientation.w));

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

    //odometry stuff
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;
    bool odomExists;
    geometry_msgs::msg::Pose currentPose;
};