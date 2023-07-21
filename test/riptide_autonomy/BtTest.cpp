#include "autonomy_test/autonomy_testing.hpp"

using namespace std::chrono_literals;

/**
 * @brief Setup function called before test cases run
 */
void BtTest::SetUp() {
    //init ros
    rclcpp::init(0, nullptr);
    toolNode = std::make_shared<BtTestTool>();
    startTime = toolNode->get_clock()->now();
}

/**
 * @brief Teardown function called after test cases run
 */
void BtTest::TearDown() {
    toolNode.reset();
    rclcpp::shutdown();
    UwrtBtNode::staticDeinit(); //needed to not thrash tf nodes by bringing up and down the ros context
}


rclcpp::Duration BtTest::testElapsed() {
    return toolNode->get_clock()->now() - startTime;
}
