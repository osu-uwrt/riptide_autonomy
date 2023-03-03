#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/TimedPublisher.hpp"


BT::NodeStatus testGetSwitchState(std::shared_ptr<BtTestTool> toolNode, bool auxIn, bool killIn, bool& auxOut, bool& killOut) {
    bool UNSPECIFIED_VAL = false;
    auto switchStateNode = toolNode->createLeafNodeFromConfig("GetSwitchState", BT::NodeConfiguration());

    std_msgs::msg::Bool
        auxMsg,
        killMsg;

    auxMsg.data = auxIn;
    killMsg.data = killIn;

    TimedPublisher<std_msgs::msg::Bool> 
        auxPub(toolNode, ROBOT_AUX_TOPIC, auxMsg, rclcpp::SensorDataQoS(), 20),
        killPub(toolNode, ROBOT_KILLED_TOPIC, killMsg, rclcpp::SensorDataQoS(), 20);

    BT::NodeStatus result = toolNode->tickUntilFinished(switchStateNode);
    auto blackboard = switchStateNode->config().blackboard;

    auxOut = getFromBlackboardWithDefault<double>(blackboard, "aux_switch_inserted", UNSPECIFIED_VAL);
    killOut = getFromBlackboardWithDefault<double>(blackboard, "kill_switch_inserted", UNSPECIFIED_VAL);

    return result;
}


TEST_F(BtTest, test_GetSwitchState_BothIn) {
    bool 
        auxIn = false, 
        killIn = false;

    auto result = testGetSwitchState(toolNode, true, true, auxIn, killIn);

    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(true, auxIn);
    ASSERT_EQ(true, killIn);

}

TEST_F(BtTest, test_GetSwitchState_AuxIn) {
    bool 
        auxIn = false, 
        killIn = false;

    auto result = testGetSwitchState(toolNode, true, false, auxIn, killIn);

    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(true, auxIn);
    ASSERT_EQ(false, killIn);

}

TEST_F(BtTest, test_GetSwitchState_KillIn) {
    bool 
        auxIn = false, 
        killIn = false;

    auto result = testGetSwitchState(toolNode, false, true, auxIn, killIn);

    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(false, auxIn);
    ASSERT_EQ(true, killIn);
}

TEST_F(BtTest, test_GetSwitchState_NoneIn) {
    bool 
        auxIn = true, 
        killIn = true;

    auto result = testGetSwitchState(toolNode, false, false, auxIn, killIn);

    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(false, auxIn);
    ASSERT_EQ(false, killIn);

}

TEST_F(BtTest, test_GetSwitchState_Failure_killUnpublished) {
    auto switchStateNode = toolNode->createLeafNodeFromConfig("GetSwitchState", BT::NodeConfiguration());
    TimedPublisher<std_msgs::msg::Bool> 
        auxPub(toolNode, ROBOT_AUX_TOPIC, std_msgs::msg::Bool(), rclcpp::SensorDataQoS(), 20);

    BT::NodeStatus result = toolNode->tickUntilFinished(switchStateNode);
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
}

TEST_F(BtTest, test_GetSwitchState_Failure_auxUnpublished) {
    auto switchStateNode = toolNode->createLeafNodeFromConfig("GetSwitchState", BT::NodeConfiguration());
    TimedPublisher<std_msgs::msg::Bool> 
        killPub(toolNode, ROBOT_KILLED_TOPIC, std_msgs::msg::Bool(), rclcpp::SensorDataQoS(), 20);

    BT::NodeStatus result = toolNode->tickUntilFinished(switchStateNode);
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
}

TEST_F(BtTest, test_GetSwitchState_Failure_bothUnpublished) {
    auto switchStateNode = toolNode->createLeafNodeFromConfig("GetSwitchState", BT::NodeConfiguration());
    BT::NodeStatus result = toolNode->tickUntilFinished(switchStateNode);
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
}
