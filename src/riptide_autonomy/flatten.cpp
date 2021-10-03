#include "states.h"
#include "behaviortree_cpp_v3/bt_factory.h"

/**
 * Script that flattens the robot
 */

using namespace BT;
using namespace states;

const char *treeFileName = "/big_flatten_tree.xml";

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "flatten");
    BehaviorTreeFactory factory;

    factory.registerNodeType<big_move_state>("BigMoveState");
    factory.registerNodeType<flatten_calculation_state>("FlattenCalculationState");

    //load tree
    std::string packagePath = ros::package::getPath("riptide_autonomy");
    Tree tree = factory.createTreeFromFile(packagePath + treeFileName);

    tree.tickRoot();
    return 0;
}