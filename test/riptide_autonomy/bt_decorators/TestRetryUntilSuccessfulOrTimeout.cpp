#include "autonomy_test/autonomy_testing.hpp"

BT::NodeStatus testRUSOT(std::shared_ptr<BtTestTool> toolNode, int milliseconds, DummyExecutionMode childMode, int childMetric, BT::NodeStatus childStatus, int& numIterations, int& millisElapsed) {
    //create dummy node to set child of retrynode
    auto dummyNode = toolNode->createDummyActionNode();
    dummyNode->configureExecution(childMode, childMetric, childStatus);
    
    //configure and create retry node
    double seconds = milliseconds / 1000.0;
    BT::NodeConfiguration cfg;
    cfg.input_ports["num_seconds"] = std::to_string(seconds);
    auto retryNode = toolNode->createDecoratorNodeFromConfig("RetryUntilSuccessfulOrTimeout", cfg, dummyNode);

    //execute the node and keep track of the time
    auto startTime = toolNode->get_clock()->now();
    auto result = toolNode->tickUntilFinished(retryNode);
    double secondsElapsed = (toolNode->get_clock()->now() - startTime).seconds();

    //set result parameters and return
    numIterations = dummyNode->getNumTicks();
    millisElapsed = (int) (secondsElapsed * 1000);
    return result;
}

TEST_F(BtTest, test_RetryUntilSuccessfulOrTimeout_success) {
    int iterations, millis;
    auto result = testRUSOT(toolNode, 1000, DummyExecutionMode::FINISH_AFTER_ITERATIONS, 5, BT::NodeStatus::SUCCESS, iterations, millis);

    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(iterations, 5);
    ASSERT_LE(millis, 1000);
}

TEST_F(BtTest, test_RetryUntilSuccessfulOrTimeout_fail_time) {
    int iterations, millis;
    auto result = testRUSOT(toolNode, 1000, DummyExecutionMode::FINISH_AFTER_TIME, 3000, BT::NodeStatus::SUCCESS, iterations, millis);

    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
    ASSERT_NEAR(millis, 1000, 3);
}

TEST_F(BtTest, test_RetryUntilSuccessfulOrTimeout_fail_child_failed) {
    int iterations, millis;
    auto result = testRUSOT(toolNode, 1000, DummyExecutionMode::FINISH_AFTER_ITERATIONS, 5, BT::NodeStatus::FAILURE, iterations, millis);

    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
    ASSERT_LE(millis, 1000);
}
