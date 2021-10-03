#include "states.h"

/**
 * Source file for the flatten state.
 */

using namespace states;
using namespace BT;

/**
 * Initializes the state.
 */
void flatten_calculation_state::init() {
    locExists = false;
    this->locSubcriber = n.subscribe(locTopic, CACHE_SIZE, &flatten_calculation_state::locCallback, this);
}

/**
 * Returns the ports that the state uses:
 * - x position
 * - y position
 * - z position
 * - x orientation
 * - y orientation
 * - z orientation
 * - w orientation
 */
PortsList flatten_calculation_state::providedPorts() {
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
 * Ticks the state.
 */
NodeStatus flatten_calculation_state::tick() {
    //wait for loc to exist
    while(!locExists) {
        ros::spinOnce();
    }

    geometry_msgs::Pose currentPose = latestLocData;
    geometry_msgs::Pose goalPose;

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

/**
 * Called when localization data comes in
 */
void flatten_calculation_state::locCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    latestLocData = msg->pose.pose;
    locExists = true;
}