#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/TimedPublisher.hpp"

TEST(BtTest, test_GetOdometry_success_zero) {
    auto odometryNode = BtTestEnvironment::getBtTestTool()->createLeafNodeFromConfig("GetOdometry", BT::NodeConfiguration());

    //define goal message
    nav_msgs::msg::Odometry odometry;
    odometry.pose.pose.position.x = 0;
    odometry.pose.pose.position.y = 0;
    odometry.pose.pose.position.z = 0;
    odometry.pose.pose.orientation.x = 0;
    odometry.pose.pose.orientation.y = 0;
    odometry.pose.pose.orientation.z = 0;
    odometry.pose.pose.orientation.w = 1; //fully zero quaternion will never happen but we doing it for the test

    TimedPublisher<nav_msgs::msg::Odometry> timedPub(BtTestEnvironment::getBtTestTool(), "odometry/filtered", odometry);

    const double UNDEFINED_VALUE = 999.99;

    //run bt node
    BT::NodeStatus result = BtTestEnvironment::getBtTestTool()->tickUntilFinished(odometryNode);
    auto blackboard = odometryNode->config().blackboard;

    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);

    double
        receivedPosX     = getFromBlackboardWithDefault<double>(blackboard, "x", UNDEFINED_VALUE),
        receivedPosY     = getFromBlackboardWithDefault<double>(blackboard, "y", UNDEFINED_VALUE),
        receivedPosZ     = getFromBlackboardWithDefault<double>(blackboard, "z", UNDEFINED_VALUE),
        receivedPosRoll  = getFromBlackboardWithDefault<double>(blackboard, "or", UNDEFINED_VALUE),
        receivedPosPitch = getFromBlackboardWithDefault<double>(blackboard, "op", UNDEFINED_VALUE),
        receivedPosYaw   = getFromBlackboardWithDefault<double>(blackboard, "oy", UNDEFINED_VALUE);

    ASSERT_NEAR(receivedPosX, odometry.pose.pose.position.x, 0.00001);
    ASSERT_NEAR(receivedPosY, odometry.pose.pose.position.y, 0.00001);
    ASSERT_NEAR(receivedPosZ, odometry.pose.pose.position.z, 0.00001);

    auto receivedRPY = toRPY(odometry.pose.pose.orientation);
    ASSERT_NEAR(receivedPosRoll, receivedRPY.x, 0.00001);
    ASSERT_NEAR(receivedPosPitch, receivedRPY.y, 0.00001);
    ASSERT_NEAR(receivedPosYaw, receivedRPY.z, 0.00001);
}

TEST(BtTest, test_GetOdometry_success_nonzero) {
    auto odometryNode = BtTestEnvironment::getBtTestTool()->createLeafNodeFromConfig("GetOdometry", BT::NodeConfiguration());

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

    const double UNDEFINED_VALUE = 999.99;

    //run bt node
    BT::NodeStatus result = BtTestEnvironment::getBtTestTool()->tickUntilFinished(odometryNode);
    auto blackboard = odometryNode->config().blackboard;

    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);

    double
        receivedPosX     = getFromBlackboardWithDefault<double>(blackboard, "x", UNDEFINED_VALUE),
        receivedPosY     = getFromBlackboardWithDefault<double>(blackboard, "y", UNDEFINED_VALUE),
        receivedPosZ     = getFromBlackboardWithDefault<double>(blackboard, "z", UNDEFINED_VALUE),
        receivedPosRoll  = getFromBlackboardWithDefault<double>(blackboard, "or", UNDEFINED_VALUE),
        receivedPosPitch = getFromBlackboardWithDefault<double>(blackboard, "op", UNDEFINED_VALUE),
        receivedPosYaw   = getFromBlackboardWithDefault<double>(blackboard, "oy", UNDEFINED_VALUE);

    ASSERT_NEAR(receivedPosX, odometry.pose.pose.position.x, 0.00001);
    ASSERT_NEAR(receivedPosY, odometry.pose.pose.position.y, 0.00001);
    ASSERT_NEAR(receivedPosZ, odometry.pose.pose.position.z, 0.00001);

    auto receivedRPY = toRPY(odometry.pose.pose.orientation);
    ASSERT_NEAR(receivedPosRoll, receivedRPY.x, 0.00001);
    ASSERT_NEAR(receivedPosPitch, receivedRPY.y, 0.00001);
    ASSERT_NEAR(receivedPosYaw, receivedRPY.z, 0.00001);
}

TEST(BtTest, test_GetOdometry_failure) {
    auto odometryNode = BtTestEnvironment::getBtTestTool()->createLeafNodeFromConfig("GetOdometry", BT::NodeConfiguration());
    auto result = BtTestEnvironment::getBtTestTool()->tickUntilFinished(odometryNode);

    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
}

