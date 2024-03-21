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
unsigned int numOccurrances(std::vector<T> vector, T obj) {
    int count = 0;
    for(unsigned int i=0; i<vector.size(); i++) {
        if(vector[i] == obj) {
            count++;
        }
    }

    return count;
}


template<typename T>
bool getOutputFromBlackboard(rclcpp::Node::SharedPtr rosnode, BT::Blackboard::Ptr bb, const std::string& key, T& value) {
    //output from node is a string value, so get it and convert it to the desired type
    std::string valStr;
    bool gotten = getFromBlackboard(rosnode, bb, key, valStr);
    if(gotten) {
        value = BT::convertFromString<T>(valStr);
        return true;
    }
    
    return false;
}


/**
 * @brief Execution modes for DummyActionNode.
 */
enum DummyExecutionMode {
    FINISH_AFTER_TIME,           // return finish status after a certain amount of time
    FINISH_AFTER_ITERATIONS,     // return finish status after a certain number of ticks
};


/**
 * @brief Action node implementation to help with the BT test suite. This node 
 * is configurable to return a specified finish status either
 *  - after a certain amount of time
 *  - after a certain number of iterations
 *  - always
 * 
 * The client can choose this by setting the execution mode and specifying the finish 
 * status. When all is specified. this node runs like a normal BT node and the client 
 * can keep track of how many times it was ticked.
 */
class DummyActionNode : public UWRTActionNode {
    public:
    DummyActionNode(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config) { }

    static BT::PortsList providedPorts() {
        return { };
    }

    void rosInit() override { }

    /**
     * @brief Configures the execution of the node.
     * To configure the node to always return a status, use configureAlwaysReturnStatus()
     * 
     * @param mode The new execution mode to use.
     * @param metric The execution metric. If mode is FINISH_AFTER_TIME, this number is 
     * the number of milliseconds after which to finish. If mode is FINISH_AFTER_ITERATIONS,
     * this number will be the number of ticks after which to finish.
     * @param status The finish status. Will be returned after the execution metric is met.
     */
    void configureExecution(DummyExecutionMode mode, int metric, BT::NodeStatus status);

    /**
     * @brief Configures the execution of the node to always return the given status
     * 
     * @param status The status to always return.
     */
    void configureAlwaysReturnStatus(BT::NodeStatus status);

    /**
     * @brief Returns the number of times the node was ticked.
     * 
     * @return int Number of ticks
     */
    int getNumTicks();

    /**
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override;

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after 
     */
    BT::NodeStatus onRunning() override;

    /**
     * @brief Called when the node is halted.
     */
    void onHalted() override { }

    private:
    DummyExecutionMode executionMode;
    BT::NodeStatus finishStatus;

    //execution vars
    int numTicks;
    rclcpp::Time startTime;

    int finishMetric; //either number of milliseconds or number of ticks depending on executionMode
};


class BtTestTool : public rclcpp::Node {
    public:
    BtTestTool();
    std::shared_ptr<BT::TreeNode> createLeafNodeFromConfig(std::string name, BT::NodeConfiguration config);
    std::shared_ptr<BT::TreeNode> createDecoratorNodeFromConfig(std::string name, BT::NodeConfiguration config, BT::TreeNode::Ptr child);
    std::shared_ptr<DummyActionNode> createDummyActionNode();
    BT::NodeStatus tickUntilFinished(std::shared_ptr<BT::TreeNode> node, const std::chrono::duration<double> &timeout = 1s);
    void spinForTime(std::chrono::duration<double> time);

    private:
    std::shared_ptr<BT::BehaviorTreeFactory> factory;
};

/**
 * @brief A ROS node for behavior tree testing. Test cases can use this to initialize UwrtBtNodes and create publishers/subscribers
 */
class BtTest : public ::testing::Test {
    public:
    static void initBtTest(int argc, char **argv);

    protected:
    void SetUp() override;
    void TearDown() override;
    rclcpp::Duration testElapsed();

    rclcpp::Time startTime;
    std::shared_ptr<BtTestTool> toolNode;

    private:
    static char **argv;
    static int argc;
};

// this class serves no purpose except to split up the regular bt test and the test tool test suites.
class TestToolTest : public BtTest { };
