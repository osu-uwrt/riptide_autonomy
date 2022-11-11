#include "autonomy_test/autonomy_testing.hpp"

BT::NodeStatus testIsTrue(bool value) {
    BT::NodeConfiguration cfg;
    cfg.input_ports["value"] = (value ? "1" : "0");
    auto node = BtTestEnvironment::getBtTestTool()->createLeafNodeFromConfig("IsTrue", cfg);
    return BtTestEnvironment::getBtTestTool()->tickUntilFinished(node);
}

TEST(BtTest, test_IsTrue_success) {
    ASSERT_EQ(testIsTrue(true), BT::NodeStatus::SUCCESS);
}

TEST(BtTest, test_IsTrue_fail) {
    ASSERT_EQ(testIsTrue(false), BT::NodeStatus::FAILURE);
}
