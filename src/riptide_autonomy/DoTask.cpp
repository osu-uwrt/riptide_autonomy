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
    factory.registerNodeType<ActuateState>("ActuateState");
    factory.registerNodeType<GetActuatorStatus>("GetActuatorStatus");
    factory.registerNodeType<WaitState>("WaitState");
    //your node type here...

    /**
     * CONDITIONS
     */
    factory.registerSimpleCondition("IsClawUnknown", ActuatorStateCheckers::isClawUnknown, { InputPort<int>("claw_state") });
    factory.registerSimpleCondition("IsClawOpen", ActuatorStateCheckers::isClawOpen, { InputPort<int>("claw_state") });
    factory.registerSimpleCondition("IsClawClosed", ActuatorStateCheckers::isClawClosed, { InputPort<int>("claw_state") });

    factory.registerSimpleCondition("IsTorpedoCharged", ActuatorStateCheckers::isTorpedoCharged, { InputPort<int>("torpedo_state") });
    factory.registerSimpleCondition("IsTorpedoFired", ActuatorStateCheckers::isTorpedoFired, { InputPort<int>("torpedo_state") });

    factory.registerSimpleCondition("IsDropperReady", ActuatorStateCheckers::isDropperReady, { InputPort<int>("dropper_state") });
    factory.registerSimpleCondition("IsDropperDropped", ActuatorStateCheckers::isDropperDropped, { InputPort<int>("dropper_state") });

    /**
     * SIMPLE ACTIONS
     * TODO: move these to own class to declutter this file
     */
    //action to simply print whatever is passed to the port
    factory.registerSimpleAction(
        "Info", 
        [] (BT::TreeNode& n) { //lambda that prints to info
            RCLCPP_INFO(log, "%s", n.getInput<std::string>("message").value().c_str()); 
            return NodeStatus::SUCCESS; 
        },

        { InputPort<std::string>("message") }
    );

    //action to print whatever is passed to the port to RCLCPP_ERROR
    factory.registerSimpleAction(
        "Error",
        [] (BT::TreeNode& n) { //lambda that prints to error
            RCLCPP_ERROR(log, "%s", n.getInput<std::string>("message").value().c_str());
            return NodeStatus::SUCCESS;
        },
        
        { InputPort<std::string>("message") }
    );

    factory.registerSimpleAction(
        "ToString",
        [] (BT::TreeNode& n) {
            BT::Optional<double> doubleIn = n.getInput<double>("double_in");
            BT::Optional<int> intIn = n.getInput<int>("int_in");

            if(doubleIn.has_value()) {
                n.setOutput<std::string>("str_out", std::to_string(doubleIn.value()));
            } else if(intIn.has_value()) {
                n.setOutput<std::string>("str_out", std::to_string(intIn.value()));
            }

            return NodeStatus::SUCCESS;
        },

        { InputPort<double>("double_in"), InputPort<int>("int_in"), OutputPort<std::string>("str_out") }
    );
    
    //figure out where the tree is based on where the package is (in ~/osu-uwrt/riptide_software/src/riptide_autonomy). Start with home
    const char *home = std::getenv("HOME");
    if(home == nullptr) {
        RCLCPP_INFO(log, "DoTask: HOME environment variable not found! Cannot continue!");
        return 1;
    }

    //create string with path to the tree
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
    PublisherZMQ zmq(tree); //publishes behaviortree data to a groot in real time

    RCLCPP_INFO(log, "DoTask: Running tree");

    NodeStatus result = tree.tickRoot();
    return (result == NodeStatus::SUCCESS ? 0 : 1); //returns 0 if success and 1 if failure
}