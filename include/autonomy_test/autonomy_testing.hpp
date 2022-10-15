#pragma once

//this header contains helpful declarations and includes for testing the UWRT behavior tree system

#include <gtest/gtest.h>
#include "riptide_autonomy/autonomy_lib.hpp"


/**
 * @brief Counts the number of elements in vector that match the object obj.
 * 
 * @tparam T The type to check
 * @param vector The vector to check
 * @param obj The item to count
 * @return int The number of times obj occurs in vector.
 */
template<typename T>
int numOccurrances(std::vector<T> vector, T obj) {
    int count = 0;
    for(unsigned int i=0; i<vector.size(); i++) {
        if(vector[i] == obj) {
            count++;
        }
    }

    return count;
}


class BtTestTool : public rclcpp::Node {
    public:
    BtTestTool();
    std::shared_ptr<BT::TreeNode> createLeafNodeFromConfig(std::string name, BT::NodeConfiguration config);
    BT::NodeStatus tickUntilFinished(std::shared_ptr<BT::TreeNode> node);

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
    std::thread executionThread;
    static std::shared_ptr<BtTestTool> toolNode;
};


