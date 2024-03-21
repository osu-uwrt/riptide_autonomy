#include "autonomy_test/autonomy_testing.hpp"

/**
 * @brief Basic test function for the Wait node.
 * 
 * @param millisecondsToWait The number of milliseconds to test the wait on.
 * @return int The number of milliseconds waited
 */
static std::tuple<BT::NodeStatus, int> WaitTest(std::shared_ptr<BtTestTool> toolNode, int millisecondsToWait) {
    //initialize wait node
    BT::NodeConfiguration in;
    in.input_ports["seconds"] = std::to_string(millisecondsToWait / 1000.0);
    auto node = toolNode->createLeafNodeFromConfig("Wait", in);

    //wait and see how long we actually waited
    rclcpp::Time startTime = toolNode->get_clock()->now();
    BT::NodeStatus status = toolNode->tickUntilFinished(node, 5s);
    rclcpp::Time finishTime = toolNode->get_clock()->now();

    int millisecondsWaited = (finishTime - startTime).seconds() * 1000;
    return std::make_tuple(status, millisecondsWaited);
}


TEST_F(BtTest, test_Wait_0s) {
    auto result = WaitTest(toolNode, 0);
    ASSERT_EQ(std::get<0>(result), BT::NodeStatus::SUCCESS);
    ASSERT_NEAR(std::get<1>(result), 0, 5);
}

TEST_F(BtTest, test_Wait_1s) {
    auto result = WaitTest(toolNode, 1000);
    ASSERT_EQ(std::get<0>(result), BT::NodeStatus::SUCCESS);
    ASSERT_NEAR(std::get<1>(result), 1000, 5);
}

TEST_F(BtTest, test_Wait_562ms) {
    auto result = WaitTest(toolNode, 562);
    ASSERT_EQ(std::get<0>(result), BT::NodeStatus::SUCCESS);
    ASSERT_NEAR(std::get<1>(result), 562, 5);
}
