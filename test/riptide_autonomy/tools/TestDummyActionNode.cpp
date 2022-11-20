#include "autonomy_test/autonomy_testing.hpp"

TEST_F(TestToolTest, test_DummyActionNode_alwaysSuccess) { //also a test for success after zero ticks
    auto dummyAction = toolNode->createDummyActionNode();
    dummyAction->configureAlwaysReturnStatus(BT::NodeStatus::SUCCESS);

    BT::NodeStatus result = toolNode->tickUntilFinished(dummyAction);
    
    ASSERT_EQ(dummyAction->getNumTicks(), 1);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
}

TEST_F(TestToolTest, test_DummyActionNode_alwaysFailure) { //also a test for failure after zero ticks
    auto dummyAction = toolNode->createDummyActionNode();
    dummyAction->configureAlwaysReturnStatus(BT::NodeStatus::FAILURE);

    BT::NodeStatus result = toolNode->tickUntilFinished(dummyAction);

    ASSERT_EQ(dummyAction->getNumTicks(), 1);
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
}

TEST_F(TestToolTest, test_DummyActionNode_successAfterNonzeroTicks) {
    auto dummyAction = toolNode->createDummyActionNode();
    dummyAction->configureExecution(DummyExecutionMode::FINISH_AFTER_ITERATIONS, 12, BT::NodeStatus::SUCCESS);

    BT::NodeStatus result = toolNode->tickUntilFinished(dummyAction);

    ASSERT_EQ(dummyAction->getNumTicks(), 12);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
}

TEST_F(TestToolTest, test_DummyActionNode_failureAfterNonzeroTicks) {
    auto dummyAction = toolNode->createDummyActionNode();
    dummyAction->configureExecution(DummyExecutionMode::FINISH_AFTER_ITERATIONS, 12, BT::NodeStatus::FAILURE);

    BT::NodeStatus result = toolNode->tickUntilFinished(dummyAction);

    ASSERT_EQ(dummyAction->getNumTicks(), 12);
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
}

TEST_F(TestToolTest, test_DummyActionNode_successAfterZeroTime) {
    auto dummyAction = toolNode->createDummyActionNode();
    dummyAction->configureExecution(DummyExecutionMode::FINISH_AFTER_TIME, 0, BT::NodeStatus::SUCCESS);
    
    BT::NodeStatus result = toolNode->tickUntilFinished(dummyAction);

    ASSERT_EQ(dummyAction->getNumTicks(), 1);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
}

TEST_F(TestToolTest, test_DummyActionNode_failureAfterZeroTime) {
    auto dummyAction = toolNode->createDummyActionNode();
    dummyAction->configureExecution(DummyExecutionMode::FINISH_AFTER_TIME, 0, BT::NodeStatus::FAILURE);
    
    BT::NodeStatus result = toolNode->tickUntilFinished(dummyAction);

    ASSERT_EQ(dummyAction->getNumTicks(), 1);
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
}

TEST_F(TestToolTest, test_DummyActionNode_successAfterNonzeroTime) {
    auto dummyAction = toolNode->createDummyActionNode();
    dummyAction->configureExecution(DummyExecutionMode::FINISH_AFTER_TIME, 2500, BT::NodeStatus::SUCCESS);

    rclcpp::Time startTime = toolNode->get_clock()->now();
    BT::NodeStatus result = toolNode->tickUntilFinished(dummyAction);
    rclcpp::Time finishTime = toolNode->get_clock()->now();
    double secondsElapsed = (finishTime - startTime).seconds();

    ASSERT_GT(dummyAction->getNumTicks(), 1);
    ASSERT_NEAR(secondsElapsed, 2.5, 0.05);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
}
