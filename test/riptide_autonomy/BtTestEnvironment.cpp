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
    if(toolNode == nullptr) { //if tool
        toolNode = std::make_shared<BtTestTool>();
    } else {
        RCLCPP_INFO(toolNode->get_logger(), "Not creating tool node because it already exists.");
    }
}

/**
 * @brief Teardown function called after test cases run
 */
void BtTestEnvironment::TearDown() {
    rclcpp::shutdown();
}

/**
 * @brief Tool node accessor
 * 
 * @return std::shared_ptr<BtTestNode> The tool node
 */
std::shared_ptr<BtTestTool> BtTestEnvironment::getBtTestTool() { //static
    return toolNode;
}
