#include "autonomy_test/autonomy_testing.hpp"

BT::NodeStatus testIsTrue(std::shared_ptr<BtTestTool> toolNode, bool value) {
    BT::NodeConfiguration cfg;
    cfg.input_ports["value"] = (value ? "1" : "0");
    auto node = toolNode->createLeafNodeFromConfig("IsTrue", cfg);
    return toolNode->tickUntilFinished(node);
}

TEST_F(BtTest, test_IsTrue_success) {
    ASSERT_EQ(testIsTrue(toolNode, true), BT::NodeStatus::SUCCESS);
}

TEST_F(BtTest, test_IsTrue_fail) {
    ASSERT_EQ(testIsTrue(toolNode, false), BT::NodeStatus::FAILURE);
}
