#include "states.h"
#include "behaviortree_cpp_v3/bt_factory.h"

/**
 * Script that moves the robot
 */

using namespace BT;
using namespace states;

//path to the tree file, relative to riptide_autonomy diretcory
const char *treeFileName = "/big_move_tree.xml";

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "big_move_state");
    BehaviorTreeFactory factory;

    factory.registerNodeType<big_move_state>("BigMoveState");

    //load tree from relative path
    std::string packagePath = ros::package::getPath("riptide_autonomy");
    Tree tree = factory.createTreeFromFile(packagePath + treeFileName);

    tree.tickRoot();
    return 0;
}