#include "autonomy_test/autonomy_testing.hpp"

double fromDegrees(double degrees)
{
    return degrees * M_PI / 180.0;
}

BT::NodeStatus approxEQAngleTest(std::shared_ptr<BtTestTool> toolNode, double a, double b, double range) {
    BT::NodeConfiguration cfg;
    cfg.input_ports["a"] = std::to_string(fromDegrees(a));
    cfg.input_ports["b"] = std::to_string(fromDegrees(b));
    cfg.input_ports["range"] = std::to_string(fromDegrees(range));

    auto node = toolNode->createLeafNodeFromConfig("ApproxEqualToAngle", cfg);
    return toolNode->tickUntilFinished(node);
}

//
// zero tests
//
TEST_F(BtTest, test_ApproxEqualToAngle_success_zero) {
    auto result = approxEQAngleTest(toolNode, 0, 0, 0.01);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
}

TEST_F(BtTest, test_ApproxEqualToAngle_failure_zero) {
    auto result = approxEQAngleTest(toolNode, 0, 1, 0.5);
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
}

//
// small tests
//
TEST_F(BtTest, test_ApproxEqualToAngle_success_small_positive) {
    auto result = approxEQAngleTest(toolNode, 45, 42, 5);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
}

TEST_F(BtTest, test_ApproxEqualToAngle_failure_small_positive) {
    auto result = approxEQAngleTest(toolNode, 45, 51, 5);
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
}

TEST_F(BtTest, test_ApproxEqualToAngle_success_small_negative) {
    auto result = approxEQAngleTest(toolNode, -104, -105, 2);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
}

TEST_F(BtTest, test_ApproxEqualToAngle_failure_small_negative) {
    auto result = approxEQAngleTest(toolNode, -104, -97, 2);
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
}

TEST_F(BtTest, test_ApproxEqualToAngle_success_small_both) {
    auto result = approxEQAngleTest(toolNode, 6, -2, 10);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
}

TEST_F(BtTest, test_ApproxEqualToAngle_success_small_both_2_swapped_order) {
    auto result = approxEQAngleTest(toolNode, -2, 6, 10);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
}

TEST_F(BtTest, test_ApproxEqualToAngle_failure_small_both) {
    auto result = approxEQAngleTest(toolNode, -3, 4, 2);
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
}

//
// big tests
//

//this one got me
TEST_F(BtTest, test_ApproxEqualToAngle_success_180) {
    auto result = approxEQAngleTest(toolNode, 160, -164, 60);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
}

TEST_F(BtTest, test_ApproxEqualToAngle_success_big_positive) {
    auto result = approxEQAngleTest(toolNode, 284, 287, 5);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
}

TEST_F(BtTest, test_ApproxEqualToAngle_success_bigger_positive) {
    auto result = approxEQAngleTest(toolNode, 754, 751, 5);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
}

TEST_F(BtTest, test_ApproxEqualToAngle_failure_big_positive) {
    auto result = approxEQAngleTest(toolNode, 284, 260, 8);
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
}

TEST_F(BtTest, test_ApproxEqualToAngle_failure_bigger_positive) {
    auto result = approxEQAngleTest(toolNode, 725, 720, 3);
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
}

TEST_F(BtTest, test_ApproxEqualToAngle_success_big_negative) {
    auto result = approxEQAngleTest(toolNode, -284, -287, 5);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
}

TEST_F(BtTest, test_ApproxEqualToAngle_success_bigger_negative) {
    auto result = approxEQAngleTest(toolNode, -724, -721, 4);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
}

TEST_F(BtTest, test_ApproxEqualToAngle_failure_big_negative) {
    auto result = approxEQAngleTest(toolNode, -284, -260, 8);
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
}

TEST_F(BtTest, test_ApproxEqualToAngle_failure_bigger_negative) {
    auto result = approxEQAngleTest(toolNode, -725, -720, 3);
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
}

TEST_F(BtTest, test_ApproxEqualToAngle_success_big_both) {
    auto result = approxEQAngleTest(toolNode, -98, 265, 5);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
}

TEST_F(BtTest, test_ApproxEqualToAngle_success_big_both_swapped_order) {
    auto result = approxEQAngleTest(toolNode, 265, -98, 5);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
}

TEST_F(BtTest, test_ApproxEqualToAngle_success_bigger_both) {
    auto result = approxEQAngleTest(toolNode, -720, 721, 5);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
}

TEST_F(BtTest, test_ApproxEqualToAngle_success_bigger_both_swapped_order) {
    auto result = approxEQAngleTest(toolNode, 721, -720, 5);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
}

TEST_F(BtTest, test_ApproxEqualToAngle_failure_big_both) {
    auto result = approxEQAngleTest(toolNode, -90, 265, 4);
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
}

TEST_F(BtTest, test_ApproxEqualToAngle_failure_bigger_both) {
    auto result = approxEQAngleTest(toolNode, -720, 725, 2);
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
}
