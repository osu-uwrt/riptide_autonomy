#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/BufferedSubscriber.hpp"

using LedCmd = riptide_msgs2::msg::LedCommand;

BT::NodeStatus testSetStatus(std::shared_ptr<BtTestTool> toolNode, const std::string& status, LedCmd& cmdOut) {
    //set up subscriber to intercept the message from the node
    BufferedSubscriber<LedCmd> bufferedSub(toolNode, LED_COMMAND_TOPIC);

    //create the node
    BT::NodeConfiguration cfg;
    cfg.input_ports["status"] = status;
    auto node = toolNode->createLeafNodeFromConfig("SetStatus", cfg);

    //tick the node
    BT::NodeStatus res = node->executeTick();
    rclcpp::spin_some(toolNode);

    if(bufferedSub.getMessages().size() > 0) {
        cmdOut = bufferedSub.getMessages()[0];
    } else {
        RCLCPP_ERROR(toolNode->get_logger(), "When testing SetStatus, expecting a message on topic %s, but none received.", LED_COMMAND_TOPIC.c_str());
    }

    return res;
}

TEST_F(BtTest, test_SetStatus_waiting) {
    riptide_msgs2::msg::LedCommand cmd;
    BT::NodeStatus result = testSetStatus(toolNode, "waiting", cmd);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(cmd.red, 255);
    ASSERT_EQ(cmd.green, 0);
    ASSERT_EQ(cmd.blue, 0);
    ASSERT_EQ(cmd.mode, LedCmd::MODE_SOLID);
    ASSERT_EQ(cmd.target, LedCmd::TARGET_ALL);
}

TEST_F(BtTest, test_SetStatus_starting) {
    riptide_msgs2::msg::LedCommand cmd;
    BT::NodeStatus result = testSetStatus(toolNode, "starting", cmd);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(cmd.red, 0);
    ASSERT_EQ(cmd.green, 0);
    ASSERT_EQ(cmd.blue, 255);
    ASSERT_EQ(cmd.mode, LedCmd::MODE_SLOW_FLASH);
    ASSERT_EQ(cmd.target, LedCmd::TARGET_ALL);
}

TEST_F(BtTest, test_SetStatus_moving) {
    riptide_msgs2::msg::LedCommand cmd;
    BT::NodeStatus result = testSetStatus(toolNode, "moving", cmd);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(cmd.red, 0);
    ASSERT_EQ(cmd.green, 255);
    ASSERT_EQ(cmd.blue, 0);
    ASSERT_EQ(cmd.mode, LedCmd::MODE_SOLID);
    ASSERT_EQ(cmd.target, LedCmd::TARGET_ALL);
}

TEST_F(BtTest, test_SetStatus_searching) {
    riptide_msgs2::msg::LedCommand cmd;
    BT::NodeStatus result = testSetStatus(toolNode, "searching", cmd);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(cmd.red, 255);
    ASSERT_EQ(cmd.green, 100);
    ASSERT_EQ(cmd.blue, 0);
    ASSERT_EQ(cmd.mode, LedCmd::MODE_SOLID);
    ASSERT_EQ(cmd.target, LedCmd::TARGET_ALL);
}

TEST_F(BtTest, test_SetStatus_aligning) {
    riptide_msgs2::msg::LedCommand cmd;
    BT::NodeStatus result = testSetStatus(toolNode, "aligning", cmd);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(cmd.red, 255);
    ASSERT_EQ(cmd.green, 0);
    ASSERT_EQ(cmd.blue, 255);
    ASSERT_EQ(cmd.mode, LedCmd::MODE_SLOW_FLASH);
    ASSERT_EQ(cmd.target, LedCmd::TARGET_ALL);
}

TEST_F(BtTest, test_SetStatus_performing) {
    riptide_msgs2::msg::LedCommand cmd;
    BT::NodeStatus result = testSetStatus(toolNode, "performing", cmd);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(cmd.red, 0);
    ASSERT_EQ(cmd.green, 0);
    ASSERT_EQ(cmd.blue, 255);
    ASSERT_EQ(cmd.mode, LedCmd::MODE_FAST_FLASH);
    ASSERT_EQ(cmd.target, LedCmd::TARGET_ALL);
}

TEST_F(BtTest, test_SetStatus_success) {
    riptide_msgs2::msg::LedCommand cmd;
    BT::NodeStatus result = testSetStatus(toolNode, "success", cmd);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(cmd.red, 0);
    ASSERT_EQ(cmd.green, 255);
    ASSERT_EQ(cmd.blue, 0);
    ASSERT_EQ(cmd.mode, LedCmd::MODE_BREATH);
    ASSERT_EQ(cmd.target, LedCmd::TARGET_ALL);
}

TEST_F(BtTest, test_SetStatus_failure) {
    riptide_msgs2::msg::LedCommand cmd;
    BT::NodeStatus result = testSetStatus(toolNode, "failure", cmd);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(cmd.red, 255);
    ASSERT_EQ(cmd.green, 0);
    ASSERT_EQ(cmd.blue, 0);
    ASSERT_EQ(cmd.mode, LedCmd::MODE_BREATH);
    ASSERT_EQ(cmd.target, LedCmd::TARGET_ALL);
}

TEST_F(BtTest, test_SetStatus_panic) {
    riptide_msgs2::msg::LedCommand cmd;
    BT::NodeStatus result = testSetStatus(toolNode, "panic", cmd);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(cmd.red, 255);
    ASSERT_EQ(cmd.green, 0);
    ASSERT_EQ(cmd.blue, 0);
    ASSERT_EQ(cmd.mode, LedCmd::MODE_FAST_FLASH);
    ASSERT_EQ(cmd.target, LedCmd::TARGET_ALL);
}

TEST_F(BtTest, test_SetStatus_undefined) {
    riptide_msgs2::msg::LedCommand cmd;
    BT::NodeStatus result = testSetStatus(toolNode, "AAAAAAAAAAAAAAAAAAAAA", cmd);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(cmd.red, 255);
    ASSERT_EQ(cmd.green, 255);
    ASSERT_EQ(cmd.blue, 255);
    ASSERT_EQ(cmd.mode, LedCmd::MODE_SOLID);
    ASSERT_EQ(cmd.target, LedCmd::TARGET_ALL);
}
