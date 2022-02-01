#include "autonomy.h"

using namespace BT;

/**
 * This is the boilerplate state template.
 * 
 * This class can be used as a template for creating a new state to be run in the BehaviorTree.
 * These are the steps:
 * - Copy this file and rename the copy to whatever your new state's name is (preferably title cased for consistency :))
 * 
 * - Change the class name to whatever you named the file to
 * 
 * - Go to autonomy.h (in the include directory) and locate the BaseState class. It should be under the UWRTSyncActionNode 
 *   declaration (use ctrl-F!).
 * 
 * - Copy the BaseState class declaration, paste it at the bottom of the header file (BEFORE the #endif though!) and follow 
 *   the steps in the BaseState comment.
 * 
 * - Go to the CMakeLists.txt and add your new file to the add_library call.
 * 
 * - Resolve any other TODOs that are not resolved, then delete this comment and implement your init() and tick() methods. 
 *   If you need to add any other public or private methods (you will if you make a Subscription), feel free to add those!
 *   Make sure you also declare the method in the autonomy.h!
 */


void BaseState::init(rclcpp::Node::SharedPtr node) { //TODO: Change BaseState to your new class name
    this->rosnode = node;

    //TODO: Define needed ros subscriptions and publishers here...
}


NodeStatus BaseState::tick() { //TODO: Change BaseState to your new class name  
    //TODO: Implement this method. It should return NodeStatus::SUCCESS or NodeStatus::FAILURE
    return NodeStatus::SUCCESS;
}
