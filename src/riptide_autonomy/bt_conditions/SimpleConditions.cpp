#include "autonomy.h"


/**
 * @brief Registers simple conditions.
 *
 * @param factory The factory to register conditions with.
 */
void SimpleConditions::registerConditions(BT::BehaviorTreeFactory *factory) {
    /**
     * Check if two numbers are equal
     * This is now deprecated. Use CompareNums instead.
     */
    factory->registerSimpleCondition(
        "NumsEqual",
        [] (BT::TreeNode& n) {
            double
                a = n.getInput<double>("a").value(),
                b = n.getInput<double>("b").value();

            RCLCPP_WARN(log, "Using deprecated state NumsEqual! Use CompareNums instead.");

            return (a == b ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE);
        },
        { BT::InputPort<double>("a"), BT::InputPort<double>("b") }
    );

    /**
     * Basic condition that checks numbers. Users can check if a number is greater than, less than, or equal to another number.
     */
    factory->registerSimpleCondition(
        "CompareNums",
        [] (BT::TreeNode& n) {
            std::string test = n.getInput<std::string>("test").value();

            double
                a = n.getInput<double>("a").value(),
                b = n.getInput<double>("b").value();

            if(test == ">") { //check a > b
                return (a > b ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE);
            } else if(test == "<") { //check if a < b
                return (a < b ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE);
            } else if(test == "=") { // check if a == b
                return (a == b ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE);
            }

            RCLCPP_ERROR(log, "Invalid operator %s!", test.c_str());
            return BT::NodeStatus::FAILURE;
        },
        {
            BT::InputPort<std::string>("test"),
            BT::InputPort<double>("a"),
            BT::InputPort<double>("b")
        }
    );

    /**
     * Basic condition that checks if two numbers are approximately equal to each other
     */
    factory->registerSimpleCondition(
        "ApproxEqualTo",
        [] (BT::TreeNode& n) {
            double
                a = n.getInput<double>("a").value(),
                b = n.getInput<double>("b").value(),
                range = n.getInput<double>("range").value();

            return (abs(a - b) < range ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE);
        },
        {
            BT::InputPort<double>("a"),
            BT::InputPort<double>("b"),
            BT::InputPort<double>("range")
        }
    );

    /**
     * Basic condition that checks if a boolean value is true
     */
    factory->registerSimpleCondition(
        "IsTrue",
        [] (BT::TreeNode& n) {
            bool value = n.getInput<bool>("value").value();
            return (value ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE);
        },
        {
            BT::InputPort<bool>("value")
        }
    );
}