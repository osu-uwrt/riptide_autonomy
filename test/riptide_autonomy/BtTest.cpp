#include "autonomy_test/autonomy_testing.hpp"

using namespace std::chrono_literals;

/**
 * @brief Setup function called before test cases run
 */
void BtTest::SetUp() {
    //init ros
    rclcpp::init(0, nullptr);

    //if tool doesn't exist, create it
    if(toolNode == nullptr) {
        toolNode = std::make_shared<BtTestTool>();

        executionThread = std::thread(
            [this] () {
                rclcpp::spin(toolNode);
            }
        );
    } else {
        RCLCPP_INFO(toolNode->get_logger(), "Not creating tool node because it already exists.");
    }
}

/**
 * @brief Teardown function called after test cases run
 */
void BtTest::TearDown() {
    while(!toolNode->isSpinning()) {
        rclcpp::sleep_for(1ms);
    }

    rclcpp::shutdown(); //will invalidate ros context and kill node

    // wait for execution thread to stop if it is still running
    if(executionThread.joinable()) {
        executionThread.join();
    }

    toolNode.reset();
}
