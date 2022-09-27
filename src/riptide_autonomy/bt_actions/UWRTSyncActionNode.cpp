#include "autonomy.h"

/**
 * @brief Source file for the UWRTSyncActionNode class.
 * This is a BehaviorTree node that can be initialized
 * with a ros node for creating publishers / subscribers
 * without creating a new node for every state change.
 * 
 */

using namespace BT;

/**
 * @brief Base providedPorts() method for action node. 
 * If inheriting class does not override this method, then this
 * will be used.
 * 
 * @return PortsList Will return no ports.
 */
PortsList UWRTSyncActionNode::providedPorts() {
    return {};
}

/**
 * @brief Base tick() method for action node. 
 * This method should be overridden and will print a message to
 * the console if it is not.
 * 
 * @return BT::NodeStatus The status of the node after tick completes.
 */
NodeStatus UWRTSyncActionNode::tick() {
    RCLCPP_WARN(log, "UWRTSyncActionNode's tick() method not overridden! This action does nothing! Override me!");
    return NodeStatus::SUCCESS;
}