#include "autonomy.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

/**
 * C++ Script that runs a given behavior tree.
 * Script will take in the file path to a behavior
 * tree XML file as an argument, then run that 
 * tree and return the result (0 if success, 1 if failure) 
 */

using namespace BT;

const char *AUTONOMY_PATH_FROM_HOME = "/osu-uwrt/riptide_software/src/riptide_autonomy/trees/";

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr rosnode = std::make_shared<rclcpp::Node>("DoTask");

    //get name of tree to run
    if(argc <= 1) {
        RCLCPP_INFO(log, "DoTask: No Tree to run. DoTask will exit.");
        return 1;
    }

    BehaviorTreeFactory factory;
    RCLCPP_INFO(log, "DoTask: Registering Nodes");

    /**
     * REGISTER NODES HERE
     */
    factory.registerNodeType<BigMoveState>("BigMoveState");
    factory.registerNodeType<FlattenCalculationState>("FlattenCalculationState");
    factory.registerNodeType<ToWorldFrameState>("ToWorldFrameState");
    factory.registerNodeType<VelocityState>("VelocityState");
    factory.registerNodeType<SearchState>("SearchState");
    factory.registerNodeType<LineDriveCalcState>("LineDriveCalcState");
    //your node type here...
    
    //figure out where the tree is based on where the package is (in ~/osu-uwrt/riptide_software/src/riptide_autonomy). Start with home
    const char *home = std::getenv("HOME");
    if(home == nullptr) {
        RCLCPP_INFO(log, "DoTask: HOME environment variable not found! Cannot continue!");
        return 1;
    }
    std::string treePath = home + std::string(AUTONOMY_PATH_FROM_HOME) + argv[1];
    RCLCPP_INFO(log, "DoTask: Creating Tree From File: %s", treePath.c_str());

    //load tree
    Tree tree = factory.createTreeFromFile(treePath);

    //initialize all nodes in tree
    for(auto& node : tree.nodes) {
        // Not a typo: it is "=", not "=="
        if( auto action = dynamic_cast<UWRTSyncActionNode*>( node.get() )) {
            action->init(rosnode);
        }
    }

    RCLCPP_INFO(log, "DoTask: Loading Monitor");
    PublisherZMQ zmq(tree);

    RCLCPP_INFO(log, "DoTask: Running tree");

    NodeStatus result = tree.tickRoot();
    return (result == NodeStatus::SUCCESS ? 0 : 1); //returns 0 if success and 1 if failure
}