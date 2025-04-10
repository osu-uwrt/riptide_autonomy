#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/TimedPublisher.hpp"

using namespace std::chrono_literals;

const std::chrono::duration<double> TESTTWIST_TIMEOUT = 5s;

BT::NodeStatus testGetTwistTopic(std::shared_ptr<BtTestTool> toolNode, const std::string& topic, const geometry_msgs::msg::Twist& valToPub, bool& outputSet, geometry_msgs::msg::Twist& receivedVal, const int pubPeriodMs = 125) {
    //set up node
    BT::NodeConfiguration cfg;
    cfg.input_ports["topic"] = topic;
    
    auto node = toolNode->createLeafNodeFromConfig("GetTwistTopic", cfg);
    
    //set up timed pub and expected msg
    TimedPublisher<geometry_msgs::msg::Twist> timedPub(toolNode, topic, valToPub, 10, pubPeriodMs);

    //run node
    BT::NodeStatus status = toolNode->tickUntilFinished(node, TESTTWIST_TIMEOUT);

    //node done, get result
    auto blackboard = node->config().blackboard;

    outputSet = true;
    outputSet = outputSet && getOutputFromBlackboard<double>(toolNode, blackboard, "vel_x", receivedVal.linear.x);
    outputSet = outputSet && getOutputFromBlackboard<double>(toolNode, blackboard, "vel_y", receivedVal.linear.y);
    outputSet = outputSet && getOutputFromBlackboard<double>(toolNode, blackboard, "vel_z", receivedVal.linear.z);
    outputSet = outputSet && getOutputFromBlackboard<double>(toolNode, blackboard, "vel_roll", receivedVal.angular.x);
    outputSet = outputSet && getOutputFromBlackboard<double>(toolNode, blackboard, "vel_pitch", receivedVal.angular.y);
    outputSet = outputSet && getOutputFromBlackboard<double>(toolNode, blackboard, "vel_yaw", receivedVal.angular.z);
    return status;
}

TEST_F(BtTest, test_GetTwistTopic_success_zero) {
    bool outSet;

    geometry_msgs::msg::Twist 
        expected,
        result;

    BT::NodeStatus stat = testGetTwistTopic(toolNode, "/some/twist", expected, outSet, result);

    ASSERT_EQ(stat, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(outSet);
    ASSERT_EQ(expected.linear.x, result.linear.x);
    ASSERT_EQ(expected.linear.y, result.linear.y);
    ASSERT_EQ(expected.angular.x, result.angular.x);
    ASSERT_EQ(expected.angular.y, result.angular.y);
    ASSERT_EQ(expected.angular.z, result.angular.z);
}

TEST_F(BtTest, test_GetTwistTopic_success_nonzero) {
    bool outSet;

    geometry_msgs::msg::Twist 
        expected,
        result;
    
    expected.linear.x = 1;
    expected.linear.y = 2;
    expected.linear.z = 3;
    expected.angular.x = 4;
    expected.angular.y = 5;
    expected.angular.z = 6;

    BT::NodeStatus stat = testGetTwistTopic(toolNode, "/some/twist", expected, outSet, result);

    ASSERT_EQ(stat, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(outSet);
    ASSERT_EQ(expected.linear.x, result.linear.x);
    ASSERT_EQ(expected.linear.y, result.linear.y);
    ASSERT_EQ(expected.angular.x, result.angular.x);
    ASSERT_EQ(expected.angular.y, result.angular.y);
    ASSERT_EQ(expected.angular.z, result.angular.z);
}


TEST_F(BtTest, test_GetTwistTopic_success_nonzero_differenttopic) {
    bool outSet;

    geometry_msgs::msg::Twist 
        expected,
        result;
    
    expected.linear.x = 6;
    expected.linear.y = 5;
    expected.linear.z = 4;
    expected.angular.x = 3;
    expected.angular.y = 2;
    expected.angular.z = 1;

    BT::NodeStatus stat = testGetTwistTopic(toolNode, "twisttopic", expected, outSet, result);

    ASSERT_EQ(stat, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(outSet);
    ASSERT_EQ(expected.linear.x, result.linear.x);
    ASSERT_EQ(expected.linear.y, result.linear.y);
    ASSERT_EQ(expected.angular.x, result.angular.x);
    ASSERT_EQ(expected.angular.y, result.angular.y);
    ASSERT_EQ(expected.angular.z, result.angular.z);
}


TEST_F(BtTest, test_GetTwistTopic_failure_nonzero_differenttopic) {
    bool outSet;

    geometry_msgs::msg::Twist 
        expected,
        result;
    
    expected.linear.x = 6;
    expected.linear.y = 5;
    expected.linear.z = 4;
    expected.angular.x = 3;
    expected.angular.y = 2;
    expected.angular.z = 1;

    BT::NodeStatus stat = testGetTwistTopic(toolNode, "twisttopic", expected, outSet, result, 5000);

    ASSERT_EQ(stat, BT::NodeStatus::FAILURE);
    ASSERT_FALSE(outSet);
}
