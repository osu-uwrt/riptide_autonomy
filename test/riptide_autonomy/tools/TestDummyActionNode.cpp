#include "autonomy_test/autonomy_testing.hpp"

TEST(TestToolTest, test_DummyActionNode_alwaysSuccess) { //also a test for success after zero ticks
    auto dummyAction = BtTestEnvironment::getBtTestTool()->createDummyActionNode();
    dummyAction->configureAlwaysReturnStatus(BT::NodeStatus::SUCCESS);

    BT::NodeStatus result = BtTestEnvironment::getBtTestTool()->tickUntilFinished(dummyAction);
    
    ASSERT_EQ(dummyAction->getNumTicks(), 1);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
}

TEST(TestToolTest, test_DummyActionNode_alwaysFailure) { //also a test for failure after zero ticks
    auto dummyAction = BtTestEnvironment::getBtTestTool()->createDummyActionNode();
    dummyAction->configureAlwaysReturnStatus(BT::NodeStatus::FAILURE);

    BT::NodeStatus result = BtTestEnvironment::getBtTestTool()->tickUntilFinished(dummyAction);

    ASSERT_EQ(dummyAction->getNumTicks(), 1);
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
}

TEST(TestToolTest, test_DummyActionNode_successAfterNonzeroTicks) {
    auto dummyAction = BtTestEnvironment::getBtTestTool()->createDummyActionNode();
    dummyAction->configureExecution(DummyExecutionMode::FINISH_AFTER_ITERATIONS, 12, BT::NodeStatus::SUCCESS);

    BT::NodeStatus result = BtTestEnvironment::getBtTestTool()->tickUntilFinished(dummyAction);

    ASSERT_EQ(dummyAction->getNumTicks(), 12);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
}

TEST(TestToolTest, test_DummyActionNode_failureAfterNonzeroTicks) {
    auto dummyAction = BtTestEnvironment::getBtTestTool()->createDummyActionNode();
    dummyAction->configureExecution(DummyExecutionMode::FINISH_AFTER_ITERATIONS, 12, BT::NodeStatus::FAILURE);

    BT::NodeStatus result = BtTestEnvironment::getBtTestTool()->tickUntilFinished(dummyAction);

    ASSERT_EQ(dummyAction->getNumTicks(), 12);
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
}

TEST(TestToolTest, test_DummyActionNode_successAfterZeroTime) {
    auto dummyAction = BtTestEnvironment::getBtTestTool()->createDummyActionNode();
    dummyAction->configureExecution(DummyExecutionMode::FINISH_AFTER_TIME, 0, BT::NodeStatus::SUCCESS);
    
    BT::NodeStatus result = BtTestEnvironment::getBtTestTool()->tickUntilFinished(dummyAction);

    ASSERT_EQ(dummyAction->getNumTicks(), 1);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
}

TEST(TestToolTest, test_DummyActionNode_failureAfterZeroTime) {
    auto dummyAction = BtTestEnvironment::getBtTestTool()->createDummyActionNode();
    dummyAction->configureExecution(DummyExecutionMode::FINISH_AFTER_TIME, 0, BT::NodeStatus::FAILURE);
    
    BT::NodeStatus result = BtTestEnvironment::getBtTestTool()->tickUntilFinished(dummyAction);

    ASSERT_EQ(dummyAction->getNumTicks(), 1);
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
}

TEST(TestToolTest, test_DummyActionNode_successAfterNonzeroTime) {
    auto dummyAction = BtTestEnvironment::getBtTestTool()->createDummyActionNode();
    dummyAction->configureExecution(DummyExecutionMode::FINISH_AFTER_TIME, 2500, BT::NodeStatus::SUCCESS);

    rclcpp::Time startTime = BtTestEnvironment::getBtTestTool()->get_clock()->now();
    BT::NodeStatus result = BtTestEnvironment::getBtTestTool()->tickUntilFinished(dummyAction);
    rclcpp::Time finishTime = BtTestEnvironment::getBtTestTool()->get_clock()->now();
    double secondsElapsed = (finishTime - startTime).seconds();

    ASSERT_GT(dummyAction->getNumTicks(), 1);
    ASSERT_NEAR(secondsElapsed, 2.5, 0.05);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
}
