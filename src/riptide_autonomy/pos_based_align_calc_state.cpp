#include "states.h"

using namespace BT;
using namespace states;

void pos_based_align_calc_state::locCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    latestLocData = msg->pose.pose;
    locExists = true;
}

PortsList pos_based_align_calc_state::providedPorts() {
    return {
        InputPort("frame"),
        InputPort("extra_distance"),
        OutputPort("x_out"),
        OutputPort("y_out"),
        OutputPort("z_out")
    };
}

NodeStatus pos_based_align_calc_state::tick() {
    locExists = false;
    ros::Subscriber locSubscriber = n.subscribe(locTopic, CACHE_SIZE, &states::pos_based_align_calc_state::locCallback, this);

    //wait for localization data
    ROS_INFO("Waiting for loc data");
    ros::Duration sleepDuration(0.25);
    while(!locExists) {
        ros::spinOnce();
        sleepDuration.sleep();
    }

    //get frame
    ROS_INFO("Starting TF2 Buffer Server");
    tf2_ros::BufferClient buffer("tf2_buffer_server");
    buffer.waitForServer();

    ROS_INFO("TF2 Buffer Server connected.");

    //look up object (should be broadcasted from mapping)
    std::string objectName = getInput<std::string>("frame").value();
    geometry_msgs::TransformStamped transform = buffer.lookupTransform("world", objectName, ros::Time(0), ros::Duration(1.0));

    geometry_msgs::Pose relativePose; //we don't want a relative pose, we want the exact thing
    geometry_msgs::Pose objectPose; // will be the exact pose of the object
    tf2::doTransform(relativePose, objectPose, transform);

    double
        deltaY = objectPose.position.y - latestLocData.position.y,
        deltaX = objectPose.position.x - latestLocData.position.x,
        yawHeading = atan2(deltaY, deltaX);

    ROS_INFO("Yaw Heading degrees: %f", yawHeading * (180/M_PI));

    //calculate birds-eye distance to target object
    double distance = sqrt((deltaY * deltaY) + (deltaX * deltaX));

    double extraDistance = std::stod(getInput<std::string>("extra_distance").value());
    double distToDrive = distance + extraDistance;

    geometry_msgs::Vector3 target;

    //convert back to a point that the robot can drive to
    target.x = distToDrive * cos(yawHeading);
    target.y = distToDrive * sin(yawHeading);
    target.z = latestLocData.position.z;

    setOutput<std::string>("x_out", std::to_string(target.x));
    setOutput<std::string>("y_out", std::to_string(target.y));
    setOutput<std::string>("z_out", std::to_string(target.z));

    return NodeStatus::SUCCESS;
}