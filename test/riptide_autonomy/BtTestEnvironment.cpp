#include "autonomy_test/autonomy_testing.hpp"

/**
 * @brief Static BtTestNode that test cases can access.
 */
std::shared_ptr<BtTestTool> BtTestEnvironment::toolNode; //static

/**
 * @brief Construct a new Bt Test Environment:: Bt Test Environment object
 * 
 * @param argc Program argument count
 * @param argv Program argument value
 */
BtTestEnvironment::BtTestEnvironment(int argc, char **argv) {
    rclcpp::init(argc, argv);
}

/**
 * @brief Setup function called before test cases run
 */
void BtTestEnvironment::SetUp() {
    //if tool doesn't exist, create it
    if(toolNode == nullptr) {
        toolNode = std::make_shared<BtTestTool>();

        executionThread = std::thread(
            [] () {
                rclcpp::spin(BtTestEnvironment::getBtTestTool());
            }
        );
    } else {
        RCLCPP_INFO(toolNode->get_logger(), "Not creating tool node because it already exists.");
    }

    std::cout << "BT Test environment successfully set up." << std::endl;
}

/**
 * @brief Teardown function called after test cases run
 */
void BtTestEnvironment::TearDown() {
    rclcpp::shutdown();

    //wait for execution thread to stop if it is still running
    if(executionThread.joinable()) {
        executionThread.join();
    }

    std::cout << "BT Test environment successfully torn down." << std::endl;
}

/**
 * @brief Tool node accessor
 * 
 * @return std::shared_ptr<BtTestNode> The tool node
 */
std::shared_ptr<BtTestTool> BtTestEnvironment::getBtTestTool() { //static
    return toolNode;
}
