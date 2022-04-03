#include "autonomy.h"

/**
 * @brief Registers simple actions to be done by the BehaviorTree.
 * 
 * @param factory The factory to register with.
 */
void SimpleStates::registerSimpleActions(BT::BehaviorTreeFactory *factory) {
    /**
     * Action to print a message to RCLCPP_INFO
     */
    factory->registerSimpleAction(
        "Info", 
        [] (BT::TreeNode& n) { //lambda that prints to info
            std::string message = n.getInput<std::string>("message").value();
            RCLCPP_INFO(log, "%s", stringWithBlackboardEntries(message, n).c_str()); 
            return NodeStatus::SUCCESS; 
        },

        { InputPort<std::string>("message") }
    );



    /**
     * Action to print a message to RCLCPP_ERROR
     */
    factory->registerSimpleAction(
        "Error",
        [] (BT::TreeNode& n) { //lambda that prints to error
            std::string message = n.getInput<std::string>("message").value();
            RCLCPP_ERROR(log, "%s", stringWithBlackboardEntries(message, n).c_str());
            return NodeStatus::SUCCESS;
        },
        
        { InputPort<std::string>("message") }
    );



    /**
     * Action to convert an int or double to a string.
     */
    factory->registerSimpleAction(
        "ToString",
        [] (BT::TreeNode& n) {
            BT::Optional<double> doubleIn = n.getInput<double>("double_in");
            BT::Optional<int> intIn = n.getInput<int>("int_in");
            if(doubleIn.has_value()) {
                n.setOutput<std::string>("str_out", std::to_string(doubleIn.value()));
            } else if(intIn.has_value()) {
                RCLCPP_INFO(log, "got intin as %i.", intIn.value());
                n.setOutput<std::string>("str_out", std::to_string(intIn.value()));
            }

            return NodeStatus::SUCCESS;
        },

        { InputPort<double>("double_in"), InputPort<int>("int_in"), OutputPort<std::string>("str_out") }
    );
}
