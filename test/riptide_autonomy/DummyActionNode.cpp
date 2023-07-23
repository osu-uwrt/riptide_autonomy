#include "autonomy_test/autonomy_testing.hpp"

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
void DummyActionNode::configureExecution(DummyExecutionMode mode, int metric, BT::NodeStatus status) {
    if(this->status() != BT::NodeStatus::RUNNING) {
        executionMode = mode;
        finishMetric = metric;
        finishStatus = status;
    } else {
        RCLCPP_ERROR(rosNode()->get_logger(), "Cannot set execution mode of DummyActionNode while it is running! Change was not applied!");
    }
}

/**
 * @brief Configures the execution of the node to always return the given status
 * 
 * @param status The status to always return.
 */
void DummyActionNode::configureAlwaysReturnStatus(BT::NodeStatus status) {
    configureExecution(DummyExecutionMode::FINISH_AFTER_ITERATIONS, 1, status);
}

/**
 * @brief Returns the number of times the node was ticked.
 * 
 * @return int Number of ticks
 */
int DummyActionNode::getNumTicks() {
    return numTicks;
}

/**
 * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
 * @return NodeStatus status of the node after execution
 */
BT::NodeStatus DummyActionNode::onStart() {
    //set up node execution
    numTicks = 0;
    startTime = rosnode->get_clock()->now();

    return onRunning();
}

/**
 * @brief Called periodically while the node status is RUNNING
 * @return NodeStatus The node status after 
 */
BT::NodeStatus DummyActionNode::onRunning() {
    numTicks++;

    switch(executionMode) {
        //return 
        case FINISH_AFTER_TIME: {
            if((rosnode->get_clock()->now() - startTime).seconds() * 1000 >= finishMetric) {
                return finishStatus;
            }
            break;
        }
        
        case FINISH_AFTER_ITERATIONS: {
            if(numTicks >= finishMetric) {
                return finishStatus;
            }
            break;
        }

        //no default, will just break out of switch and return RUNNING
    }

    return BT::NodeStatus::RUNNING;
}
