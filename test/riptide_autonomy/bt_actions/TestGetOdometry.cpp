#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/TimedPublisher.hpp"

BT::NodeStatus testGetOdometry(
    std::shared_ptr<BtTestTool> toolNode, 
    nav_msgs::msg::Odometry in, 
    bool& outputsSet, 
    geometry_msgs::msg::Vector3& positionOut,
    geometry_msgs::msg::Vector3& orientationOut) 
{
    auto odometryNode = toolNode->createLeafNodeFromConfig("GetOdometry", BT::NodeConfiguration());
    TimedPublisher<nav_msgs::msg::Odometry> timedPub(toolNode, "odometry/filtered", in);

    //run the node
    BT::NodeStatus result = toolNode->tickUntilFinished(odometryNode);

    //collect results
    auto blackboard = odometryNode->config().blackboard;
    outputsSet = true;
    outputsSet = outputsSet && getOutputFromBlackboard<double>(toolNode, blackboard, "x", positionOut.x);
    outputsSet = outputsSet && getOutputFromBlackboard<double>(toolNode, blackboard, "y", positionOut.y);
    outputsSet = outputsSet && getOutputFromBlackboard<double>(toolNode, blackboard, "z", positionOut.z);
    outputsSet = outputsSet && getOutputFromBlackboard<double>(toolNode, blackboard, "or", orientationOut.x);
    outputsSet = outputsSet && getOutputFromBlackboard<double>(toolNode, blackboard, "op", orientationOut.y);
    outputsSet = outputsSet && getOutputFromBlackboard<double>(toolNode, blackboard, "oy", orientationOut.z);

    return result;
}

TEST_F(BtTest, test_GetOdometry_success_zero) {
    //define goal message
    nav_msgs::msg::Odometry odometry;
    odometry.pose.pose.position.x = 0;
    odometry.pose.pose.position.y = 0;
    odometry.pose.pose.position.z = 0;
    odometry.pose.pose.orientation.x = 0;
    odometry.pose.pose.orientation.y = 0;
    odometry.pose.pose.orientation.z = 0;
    odometry.pose.pose.orientation.w = 1; //fully zero quaternion will never happen but we doing it for the test

    bool outputsSet;
    geometry_msgs::msg::Vector3 outXyz, outRpy;
    BT::NodeStatus result = testGetOdometry(toolNode, odometry, outputsSet, outXyz, outRpy);

    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(outputsSet);

    ASSERT_NEAR(outXyz.x, odometry.pose.pose.position.x, 0.00001);
    ASSERT_NEAR(outXyz.y, odometry.pose.pose.position.y, 0.00001);
    ASSERT_NEAR(outXyz.z, odometry.pose.pose.position.z, 0.00001);

    auto receivedRPY = toRPY(odometry.pose.pose.orientation);
    ASSERT_NEAR(outRpy.x, receivedRPY.x, 0.00001);
    ASSERT_NEAR(outRpy.y, receivedRPY.y, 0.00001);
    ASSERT_NEAR(outRpy.z, receivedRPY.z, 0.00001);
}

TEST_F(BtTest, test_GetOdometry_success_nonzero) {
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
    
    bool outputsSet;
    geometry_msgs::msg::Vector3 outXyz, outRpy;
    BT::NodeStatus result = testGetOdometry(toolNode, odometry, outputsSet, outXyz, outRpy);

    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(outputsSet);

    ASSERT_NEAR(outXyz.x, odometry.pose.pose.position.x, 0.00001);
    ASSERT_NEAR(outXyz.y, odometry.pose.pose.position.y, 0.00001);
    ASSERT_NEAR(outXyz.z, odometry.pose.pose.position.z, 0.00001);

    auto receivedRPY = toRPY(odometry.pose.pose.orientation);
    ASSERT_NEAR(outRpy.x, receivedRPY.x, 0.00001);
    ASSERT_NEAR(outRpy.y, receivedRPY.y, 0.00001);
    ASSERT_NEAR(outRpy.z, receivedRPY.z, 0.00001);
}

TEST_F(BtTest, test_GetOdometry_failure_timeout) {
    auto odometryNode = toolNode->createLeafNodeFromConfig("GetOdometry", BT::NodeConfiguration());
    auto result = toolNode->tickUntilFinished(odometryNode);

    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
}

