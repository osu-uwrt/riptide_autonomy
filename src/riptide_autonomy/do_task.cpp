#include "states.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

/**
 * Script that moves the robot
 */

using namespace BT;
using namespace states;

//path to the tree file, relative to riptide_autonomy diretcory

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "task");

    //get name of tree to run
    if(argc <= 1) {
        ROS_ERROR("No Tree to run. do_task will exit.");
        return 1;
    }

    std::string treeFileName = argv[1];
    ROS_INFO("Running behavior tree %s", treeFileName.c_str());

    BehaviorTreeFactory factory;

    ROS_INFO("Registering Nodes");

    factory.registerNodeType<big_move_state>("BigMoveState");
    factory.registerNodeType<flatten_calculation_state>("FlattenCalculationState");
    factory.registerNodeType<to_world_frame_state>("toWorldFrameState");
    factory.registerNodeType<velocity_state>("VelocityState");
    factory.registerNodeType<search_state>("SearchState");
    //factory.registerNodeType<torpedo_align_state>("TorpedoAlignState");

    ROS_INFO("Creating tree");

    //load tree from relative path
    std::string packagePath = ros::package::getPath("riptide_autonomy");

    std::string treePath = packagePath + "/" + treeFileName;
    Tree tree = factory.createTreeFromFile(treePath);

    ROS_INFO("Loading Monitor");
    PublisherZMQ publisher_zmq(tree);

    ROS_INFO("Running tree");

    tree.tickRoot();
    return 0;
}