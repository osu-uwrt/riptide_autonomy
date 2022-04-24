#include "autonomy.h"

/**
 * @brief Registers simple conditions.
 * 
 * @param factory The factory to register conditions with.
 */
void SimpleConditions::registerConditions(BT::BehaviorTreeFactory *factory) {
    //check ints equal
    factory->registerSimpleCondition(
        "NumsEqual",
        [] (BT::TreeNode& n) {
            double
                a = n.getInput<double>("a").value(),
                b = n.getInput<double>("b").value();
            
            return (a == b ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE);
        },
        { InputPort<double>("a"), InputPort<double>("b") }
    );
}