#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/TimedPublisher.hpp"

TEST(BtTest, test_GetOdometry_success) {
    auto odometryNode = BtTestEnvironment::getBtTestTool()->createLeafNodeFromConfig("odometry/filtered", BT::NodeConfiguration());

    //define goal message
    geometry_msgs::msg::Vector3 rpy;
    rpy.x = 6.7;
    rpy.y = -0.2;
    rpy.z = 123.4;

    nav_msgs::msg::Odometry odometry;
    odometry.pose.pose.position.x = 8.2;
    odometry.pose.pose.position.y = 2.4;
    odometry.pose.pose.position.z = -4.5;
    odometry.pose.pose.orientation = toQuat(rpy);
    
    //set up timed pub
    TimedPublisher<nav_msgs::msg::Odometry> timedPub(BtTestEnvironment::getBtTestTool(), "odometry/filtered", odometry);

    double UNDEFINED_VALUE = 999.99;

    //run bt node
    BT::NodeStatus result = BtTestEnvironment::getBtTestTool()->tickUntilFinished(odometryNode);
    auto blackboard = odometryNode->config().blackboard;

    double
        receivedPosX = getFromBlackboardWithDefault<double>(blackboard, "x", UNDEFINED_VALUE),
        receivedPosY = getFromBlackboardWithDefault<double>(blackboard, "y", UNDEFINED_VALUE);
}

//TODO: define more tests here
