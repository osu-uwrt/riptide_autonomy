#pragma once

//this header contains helpful declarations and includes for testing the UWRT behavior tree system

#include <gtest/gtest.h>
#include "riptide_autonomy/autonomy_lib.hpp"

class BtTestTool : public rclcpp::Node {
    public:
    BtTestTool();
    BT::Blackboard::Ptr runLeafNodeFromConfig(std::string name, BT::NodeConfiguration config);

    private:
    std::shared_ptr<BT::BehaviorTreeFactory> factory;
};

/**
 * @brief A ROS node for behavior tree testing. Test cases can use this to initialize UwrtBtNodes and create publishers/subscribers
 */
class BtTestEnvironment : public ::testing::Environment {
    public:
    BtTestEnvironment(int argc, char **argv);
    void SetUp() override;
    void TearDown() override;

    static std::shared_ptr<BtTestTool> getBtTestTool();

    private:
    static std::shared_ptr<BtTestTool> toolNode;
};


