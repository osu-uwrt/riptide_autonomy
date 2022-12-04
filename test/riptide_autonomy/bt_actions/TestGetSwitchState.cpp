#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/TimedPublisher.hpp"



TEST_F(BtTest, test_GetSwitchState_BothIn) {
    bool UNSPECIFIED_VAL = false;
    auto switchStateNode = toolNode->createLeafNodeFromConfig("GetSwitchState", BT::NodeConfiguration());

    riptide_msgs2::msg::RobotState msg;
    msg.aux_switch_inserted = true;
    msg.kill_switch_inserted = true;

    TimedPublisher<riptide_msgs2::msg::RobotState> timedPub(toolNode, ROBOT_STATE_TOPIC, msg);

    BT::NodeStatus result = toolNode->tickUntilFinished(switchStateNode);
    auto blackboard = switchStateNode->config().blackboard;

    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);

    bool auxIn =  getFromBlackboardWithDefault<double>(blackboard, "aux_switch_inserted",UNSPECIFIED_VAL);
    bool killIn =  getFromBlackboardWithDefault<double>(blackboard, "kill_switch_inserted",UNSPECIFIED_VAL);

    ASSERT_EQ(true, auxIn);
    ASSERT_EQ(true, killIn);

}

TEST_F(BtTest, test_GetSwitchState_AuxIn) {
    bool UNSPECIFIED_VAL = 5;
    auto switchStateNode = toolNode->createLeafNodeFromConfig("GetSwitchState", BT::NodeConfiguration());

    riptide_msgs2::msg::RobotState msg;
    msg.aux_switch_inserted = true;
    msg.kill_switch_inserted = false;

    TimedPublisher<riptide_msgs2::msg::RobotState> timedPub(toolNode, ROBOT_STATE_TOPIC, msg);

    BT::NodeStatus result = toolNode->tickUntilFinished(switchStateNode);
    auto blackboard = switchStateNode->config().blackboard;

    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);

    bool auxIn =  getFromBlackboardWithDefault<double>(blackboard, "aux_switch_inserted",UNSPECIFIED_VAL);
    bool killIn =  getFromBlackboardWithDefault<double>(blackboard, "kill_switch_inserted",UNSPECIFIED_VAL);

    ASSERT_EQ(true, auxIn);
    ASSERT_EQ(false, killIn);

}

TEST_F(BtTest, test_GetSwitchState_KillIn) {
    bool UNSPECIFIED_VAL = false;
    auto switchStateNode = toolNode->createLeafNodeFromConfig("GetSwitchState", BT::NodeConfiguration());

    riptide_msgs2::msg::RobotState msg;
    msg.aux_switch_inserted = false;
    msg.kill_switch_inserted = true;

    TimedPublisher<riptide_msgs2::msg::RobotState> timedPub(toolNode, ROBOT_STATE_TOPIC, msg);

    BT::NodeStatus result = toolNode->tickUntilFinished(switchStateNode);
    auto blackboard = switchStateNode->config().blackboard;

    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);

    bool auxIn =  getFromBlackboardWithDefault<double>(blackboard, "aux_switch_inserted",UNSPECIFIED_VAL);
    bool killIn =  getFromBlackboardWithDefault<double>(blackboard, "kill_switch_inserted",UNSPECIFIED_VAL);

    ASSERT_EQ(false, auxIn);
    ASSERT_EQ(true, killIn);

}

TEST_F(BtTest, test_GetSwitchState_NoneIn) {
    bool UNSPECIFIED_VAL = false;
    auto switchStateNode = toolNode->createLeafNodeFromConfig("GetSwitchState", BT::NodeConfiguration());

    riptide_msgs2::msg::RobotState msg;
    msg.aux_switch_inserted = false;
    msg.kill_switch_inserted = false;

    TimedPublisher<riptide_msgs2::msg::RobotState> timedPub(toolNode, ROBOT_STATE_TOPIC, msg);

    BT::NodeStatus result = toolNode->tickUntilFinished(switchStateNode);
    auto blackboard = switchStateNode->config().blackboard;

    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);

    bool auxIn =  getFromBlackboardWithDefault<double>(blackboard, "aux_switch_inserted",UNSPECIFIED_VAL);
    bool killIn =  getFromBlackboardWithDefault<double>(blackboard, "kill_switch_inserted",UNSPECIFIED_VAL);

    ASSERT_EQ(false, auxIn);
    ASSERT_EQ(false, killIn);

}

TEST_F(BtTest, test_GetSwitchState_Failure) {

    auto switchStateNode = toolNode->createLeafNodeFromConfig("GetSwitchState", BT::NodeConfiguration());
    BT::NodeStatus result = toolNode->tickUntilFinished(switchStateNode);

    ASSERT_EQ(result, BT::NodeStatus::FAILURE);

}


