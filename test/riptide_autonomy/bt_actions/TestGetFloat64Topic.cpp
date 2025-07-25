#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/TimedPublisher.hpp"

using namespace std::chrono_literals;

const std::chrono::duration<double> TESTFLOAT_TIMEOUT = 5s;

BT::NodeStatus testGetFloat64Topic(std::shared_ptr<BtTestTool> toolNode, const std::string& topic, const double valToPub, bool& outputSet, double& receivedVal, const int pubPeriodMs = 125) {
    //set up node
    BT::NodeConfiguration cfg;
    cfg.input_ports["topic"] = topic;
    
    auto node = toolNode->createLeafNodeFromConfig("GetFloat64Topic", cfg);
    
    //set up timed pub and expected msg
    std_msgs::msg::Float64 floatMsg;
    floatMsg.data = valToPub;
    TimedPublisher<std_msgs::msg::Float64> timedPub(toolNode, topic, floatMsg, 10, pubPeriodMs);

    //run node
    BT::NodeStatus status = toolNode->tickUntilFinished(node, TESTFLOAT_TIMEOUT);

    //node done, get result
    outputSet = getOutputFromBlackboard<double>(toolNode, node->config().blackboard, "value", receivedVal);
    return status;
}

TEST_F(BtTest, test_GetFloat64Topic_zero_success) {
    bool outSet;
    double result;
    BT::NodeStatus stat = testGetFloat64Topic(toolNode, "/some/float", 0, outSet, result);

    ASSERT_EQ(stat, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(outSet);
    ASSERT_NEAR(result, 0, 0.001);
}

TEST_F(BtTest, test_GetFloat64Topic_pi_success) {
    bool outSet;
    double result;
    BT::NodeStatus stat = testGetFloat64Topic(toolNode, "/some/float", 3.1415, outSet, result);

    ASSERT_EQ(stat, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(outSet);
    ASSERT_NEAR(result, 3.1415, 0.001);
}

TEST_F(BtTest, test_GetFloat64Topic_zero_success_another_topic) {
    bool outSet;
    double result;
    BT::NodeStatus stat = testGetFloat64Topic(toolNode, "another_topic", 0, outSet, result);

    ASSERT_EQ(stat, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(outSet);
    ASSERT_NEAR(result, 0, 0.001);
}

TEST_F(BtTest, test_GetFloat64Topic_pi_success_another_topic) {
    bool outSet;
    double result;
    BT::NodeStatus stat = testGetFloat64Topic(toolNode, "another_topic", 3.1415, outSet, result);

    ASSERT_EQ(stat, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(outSet);
    ASSERT_NEAR(result, 3.1415, 0.001);
}

TEST_F(BtTest, test_GetFloat64Topic_fail_timeout) {
    bool outSet;
    double result = 0;
    BT::NodeStatus stat = testGetFloat64Topic(toolNode, "/some/float", 3.1415, outSet, result, 3500);

    ASSERT_EQ(stat, BT::NodeStatus::FAILURE);
    ASSERT_TRUE(outSet);
    ASSERT_NEAR(result, 0, 0.001);
}
