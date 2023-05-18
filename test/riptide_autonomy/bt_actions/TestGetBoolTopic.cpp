#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/TimedPublisher.hpp"

using namespace std::chrono_literals;

const std::chrono::duration<double> TESTBOOL_TIMEOUT = 5s;

BT::NodeStatus testGetBoolTopic(std::shared_ptr<BtTestTool> toolNode, const std::string& topic, const bool valToPub, bool& receivedVal, const int pubPeriodMs = 125) {
    //set up node
    BT::NodeConfiguration cfg;
    cfg.input_ports["topic"] = topic;
    
    auto node = toolNode->createLeafNodeFromConfig("GetBoolTopic", cfg);
    
    //set up timed pub and expected msg
    std_msgs::msg::Bool boolMsg;
    boolMsg.data = valToPub;
    TimedPublisher<std_msgs::msg::Bool> timedPub(toolNode, topic, boolMsg, 10, pubPeriodMs);

    //run node
    BT::NodeStatus status = BT::NodeStatus::IDLE;    
    rclcpp::Time startTime = toolNode->get_clock()->now();
    while(status != BT::NodeStatus::SUCCESS && status != BT::NodeStatus::FAILURE && toolNode->get_clock()->now() - startTime < TESTBOOL_TIMEOUT) {
        status = node->executeTick();
        rclcpp::spin_some(toolNode);
    } //if timed out, status will be RUNNING

    //node done, get result
    bool gotFromBb = getFromBlackboard(*node, "value", receivedVal);
    if(!gotFromBb) {
        RCLCPP_ERROR(log, "Could not get value from blackboard!");
        return BT::NodeStatus::IDLE;
    }

    return status;
}

TEST_F(BtTest, test_GetBoolTopic_true_success) {
    bool result;
    BT::NodeStatus stat = testGetBoolTopic(toolNode, "/some/bool", true, result);

    ASSERT_EQ(stat, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(result);
}

TEST_F(BtTest, test_GetBoolTopic_false_success) {
    bool result;
    BT::NodeStatus stat = testGetBoolTopic(toolNode, "/some/bool", false, result);

    ASSERT_EQ(stat, BT::NodeStatus::SUCCESS);
    ASSERT_FALSE(result);
}

TEST_F(BtTest, test_GetBoolTopic_true_success_another_topic) {
    bool result;
    BT::NodeStatus stat = testGetBoolTopic(toolNode, "another_topic", false, result);

    ASSERT_EQ(stat, BT::NodeStatus::SUCCESS);
    ASSERT_FALSE(result);
}

TEST_F(BtTest, test_GetBoolTopic_false_success_another_topic) {
    bool result;
    BT::NodeStatus stat = testGetBoolTopic(toolNode, "another_topic", false, result);

    ASSERT_EQ(stat, BT::NodeStatus::SUCCESS);
    ASSERT_FALSE(result);
}

TEST_F(BtTest, test_GetBoolTopic_fail_timeout) {
    bool result;
    BT::NodeStatus stat = testGetBoolTopic(toolNode, "/some/bool", true, result, 3500);

    ASSERT_EQ(stat, BT::NodeStatus::FAILURE);
    ASSERT_FALSE(result);
}
