#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>

#include <rclcpp/rclcpp.hpp>

#include "UwrtBtNode.hpp"

/**
 * C++ Script that runs a given behavior tree.
 * Script will take in the file path to a behavior
 * tree XML file as an argument, then run that
 * tree and return the result (0 if success, 1 if failure)
 */

//define logger for RCLCPP_INFO, RCLCPP_WARN, and RCLCPP_ERROR
#define log rclcpp::get_logger("autonomy")

using namespace BT;

const char *AUTONOMY_PATH_FROM_HOME = "/osu-uwrt/riptide_software/src/riptide_autonomy/";

//used std::string for str because otherwise I got an iso c++ warning
int indexOfStr(char *arr[], std::string str, int arrLen) {
    for(int i=0; i<arrLen; i++) {
        if(arr[i] == str) {
            return i;
        }
    }

    return -1;
}

std::string getEnvironmentVariable(const char *name) {
    const char *env = std::getenv(name);
    if(env == nullptr) {
        RCLCPP_INFO(log, "DoTask: %s environment variable not found!", name);
        return "";
    }

    return env;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr rosnode = std::make_shared<rclcpp::Node>("DoTask");

    BehaviorTreeFactory factory;
    RCLCPP_INFO(log, "DoTask: Registering Nodes");

    // load the plugin .so files

    //create string with path to the tree
    std::string treePath = argv[1];
    RCLCPP_INFO(log, "DoTask: Creating Tree From File: %s", treePath.c_str());

    //load tree
    Tree tree = factory.createTreeFromFile(treePath);

    //initialize all nodes in tree
    for(auto& node : tree.nodes) {
        // Not a typo: it is "=", not "=="
        if( auto btNode = dynamic_cast<UwrtBtNode*>( node.get() )) {
            btNode->init(rosnode);
        }
    }

    std::string logFile = getEnvironmentVariable("HOME") + std::string(AUTONOMY_PATH_FROM_HOME) + "btLog.fbl";
    int logArgIndex = indexOfStr(argv, "--log-file", argc);
    if(logArgIndex > -1 && logArgIndex + 1 < argc) {
        logFile = argv[logArgIndex + 1];
    }

    //load monitors and loggers

    RCLCPP_INFO(log, "DoTask: Loading Monitor");
    PublisherZMQ zmq(tree); //publishes behaviortree data to a groot in real time
    FileLogger fileLogger(tree, logFile.c_str());
    StdCoutLogger coutLogger(tree);

    bool coutEnabled = indexOfStr(argv, "--log-cout", argc) > -1;
    coutLogger.setEnabled(coutEnabled);
    if(coutEnabled) {
        RCLCPP_INFO(log, "DoTask: Cout Logging Enabled.");
    }

    RCLCPP_INFO(log, "DoTask: Running tree");

    NodeStatus result = tree.tickRoot();
    fileLogger.flush();
    coutLogger.flush(); //will flush if enabled

    RCLCPP_INFO(log, "Tree returned with %s", (result == NodeStatus::SUCCESS ? "SUCCESS" : "FAILED"));
    rclcpp::shutdown();
    return (result == NodeStatus::SUCCESS ? 0 : 1); //returns 0 if success and 1 if failure
}