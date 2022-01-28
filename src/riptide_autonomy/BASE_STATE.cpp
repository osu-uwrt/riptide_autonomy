#include "autonomy.h"

using namespace BT;

/**
 * @brief Base state template.
 * 
 * This class can be used as a template for creating a new state to be run in the BehaviorTree.
 * These are the steps:
 * - TODO: do steps
 * 
 */
class BaseState : public UWRTSyncActionNode {
    public:

    BaseState(const std::string& name, const NodeConfiguration& config)
     : UWRTSyncActionNode(name, config) { }

    /**
     * @brief Initializes the node. 
     * 
     * @param node The ROS node belonging to the current process.
     */
    void init(rclcpp::Node::SharedPtr node) {
        rosnode = node;

        //any publishers or subscribers should be initialized here...
    }

    /**
     * @brief Declares ports needed by this state.
     * 
     * @return PortsList Needed ports.
     */
    static PortsList providedPorts() {
        return {
            //ports go here...
        };
    }

    /**
     * @brief Executes the node.
     * This method will be called once by the tree and can block for as long
     * as it needs for the action to be completed. When execution completes,
     * this method must return either SUCCESS or FAILURE; it CANNOT return 
     * IDLE or RUNNING.
     * 
     * @return NodeStatus The result of the execution; SUCCESS or FAILURE.
     */
    NodeStatus tick() override {
        //main execution goes here...
    }

    private:
    rclcpp::Node::SharedPtr rosnode;
};