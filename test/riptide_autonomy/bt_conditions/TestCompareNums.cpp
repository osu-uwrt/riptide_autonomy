#include "autonomy_test/autonomy_testing.hpp"

BT::NodeStatus compareNumsTest(double a, double b, std::string operation) {
    BT::NodeConfiguration cfg;
    cfg.input_ports["a"] = std::to_string(a);
    cfg.input_ports["b"] = std::to_string(b);
    cfg.input_ports["test"] = operation;

    auto node = BtTestEnvironment::getBtTestTool()->createLeafNodeFromConfig("CompareNums", cfg);
    return BtTestEnvironment::getBtTestTool()->tickUntilFinished(node);
}

TEST(BtTest, test_CompareNums_greaterthan_success) {
    auto result = compareNumsTest(2, 0, ">");
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
}

TEST(BtTest, test_CompareNums_greaterthan_fail) {
    auto result = compareNumsTest(-2, -1, ">");
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
}

TEST(BtTest, test_CompareNums_greaterthan_fail_zero) {
    auto result = compareNumsTest(0, 0, ">");
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
}

TEST(BtTest, test_CompareNums_lessthan_success) {
    auto result = compareNumsTest(6, 6.5, "<");
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
}

TEST(BtTest, test_CompareNums_lessthan_fail) {
    auto result = compareNumsTest(0, -1, "<");
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
}

TEST(BtTest, test_CompareNums_lessthan_fail_zero) {
    auto result = compareNumsTest(0, 0, "<");
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
}

TEST(BtTest, test_CompareNums_eqaulto_success) {
    auto result = compareNumsTest(9, 9, "==");
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
}

TEST(BtTest, test_CompareNums_equalto_fail) {
    auto result = compareNumsTest(0.25, 0.6, "==");
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
}

TEST(BtTest, test_CompareNums_invalid_op) {
    auto result = compareNumsTest(2, 0, "bruh");
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
}
