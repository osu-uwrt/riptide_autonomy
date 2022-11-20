#include "autonomy_test/autonomy_testing.hpp"

BT::NodeStatus approxEQTest(std::shared_ptr<BtTestTool> toolNode, double a, double b, double range) {
    BT::NodeConfiguration cfg;
    cfg.input_ports["a"] = std::to_string(a);
    cfg.input_ports["b"] = std::to_string(b);
    cfg.input_ports["range"] = std::to_string(range);

    auto node = toolNode->createLeafNodeFromConfig("ApproxEqualTo", cfg);
    return toolNode->tickUntilFinished(node);
}

TEST_F(BtTest, test_ApproxEqualTo_success_zero) {
    auto result = approxEQTest(toolNode, 0, 0.001, 0.01);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
}

TEST_F(BtTest, test_ApproxEqualTo_success_nonzero) {
    auto result = approxEQTest(toolNode, 56.2, 56.3, 0.11);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
}

TEST_F(BtTest, test_ApproxEqualTo_fail_zero) {
    auto result = approxEQTest(toolNode, 0, 0.01, 0.0);
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
}

TEST_F(BtTest, test_ApproxEqualTo_fail_nonzero) {
    auto result = approxEQTest(toolNode, -2.1, 1.2, 0.01);
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
}
