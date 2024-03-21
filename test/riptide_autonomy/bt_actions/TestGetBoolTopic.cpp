#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/TimedPublisher.hpp"

using namespace std::chrono_literals;

const std::chrono::duration<double> TESTBOOL_TIMEOUT = 5s;

BT::NodeStatus testGetBoolTopic(std::shared_ptr<BtTestTool> toolNode, const std::string& topic, const bool valToPub, bool& outputSet, bool& receivedVal, const int pubPeriodMs = 125) {
    //set up node
    BT::NodeConfiguration cfg;
    cfg.input_ports["topic"] = topic;
    
    auto node = toolNode->createLeafNodeFromConfig("GetBoolTopic", cfg);
    
    //set up timed pub and expected msg
    std_msgs::msg::Bool boolMsg;
    boolMsg.data = valToPub;
    TimedPublisher<std_msgs::msg::Bool> timedPub(toolNode, topic, boolMsg, 10, pubPeriodMs);

    //run node
    BT::NodeStatus status = toolNode->tickUntilFinished(node, TESTBOOL_TIMEOUT);

    //node done, get result
    outputSet = getOutputFromBlackboard<bool>(toolNode, node->config().blackboard, "value", receivedVal);
    return status;
}

TEST_F(BtTest, test_GetBoolTopic_true_success) {
    bool outSet, result;
    BT::NodeStatus stat = testGetBoolTopic(toolNode, "/some/bool", true, outSet, result);

    ASSERT_EQ(stat, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(outSet);
    ASSERT_TRUE(result);
}

TEST_F(BtTest, test_GetBoolTopic_false_success) {
    bool outSet, result;
    BT::NodeStatus stat = testGetBoolTopic(toolNode, "/some/bool", false, outSet, result);

    ASSERT_EQ(stat, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(outSet);
    ASSERT_FALSE(result);
}

TEST_F(BtTest, test_GetBoolTopic_true_success_another_topic) {
    bool outSet, result;
    BT::NodeStatus stat = testGetBoolTopic(toolNode, "another_topic", false, outSet, result);

    ASSERT_EQ(stat, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(outSet);
    ASSERT_FALSE(result);
}

TEST_F(BtTest, test_GetBoolTopic_false_success_another_topic) {
    bool outSet, result;
    BT::NodeStatus stat = testGetBoolTopic(toolNode, "another_topic", false, outSet, result);

    ASSERT_EQ(stat, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(outSet);
    ASSERT_FALSE(result);
}

TEST_F(BtTest, test_GetBoolTopic_fail_timeout) {
    bool outSet, result;
    BT::NodeStatus stat = testGetBoolTopic(toolNode, "/some/bool", true, outSet, result, 3500);

    ASSERT_EQ(stat, BT::NodeStatus::FAILURE);
    ASSERT_TRUE(outSet);
    ASSERT_FALSE(result);
}
