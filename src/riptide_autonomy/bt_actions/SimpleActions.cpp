#include "autonomy.h"


/**
 * @brief Prints a string to the rclcpp info console.
 * 
 * @param n The BehaviorTree node.
 * @return BT::NodeStatus return status.
 */
BT::NodeStatus printInfo(BT::TreeNode& n) {
    std::string message = n.getInput<std::string>("message").value();
    RCLCPP_INFO(log, "%s", stringWithBlackboardEntries(message, n).c_str()); 
    return NodeStatus::SUCCESS; 
}

/**
 * @brief Prints a string to the rclcpp error console.
 * 
 * @param n The BehaviorTree node.
 * @return BT::NodeStatus return status.
 */
BT::NodeStatus printError(BT::TreeNode& n) {
    std::string message = n.getInput<std::string>("message").value();
    RCLCPP_ERROR(log, "%s", stringWithBlackboardEntries(message, n).c_str());
    return NodeStatus::SUCCESS;
}

/**
 * @brief Converts a double or integer to a string and returns it to the appropriate port.
 * 
 * @param n The BehaviorTree node.
 * @return BT::NodeStatus return status
 */
BT::NodeStatus numToString(BT::TreeNode& n) {
    BT::Optional<double> doubleIn = n.getInput<double>("double_in");
    BT::Optional<int> intIn = n.getInput<int>("int_in");
    if(doubleIn.has_value()) {
        n.setOutput<std::string>("str_out", std::to_string(doubleIn.value()));
    } else if(intIn.has_value()) {
        RCLCPP_INFO(log, "got intin as %i.", intIn.value());
        n.setOutput<std::string>("str_out", std::to_string(intIn.value()));
    }

    return NodeStatus::SUCCESS;
}

/**
 * @brief Calculates the distance between two 3d points and returns it to the appropriate port.
 * 
 * @param n The BehaviorTree node.
 * @return BT::NodeStatus return status
 */
BT::NodeStatus calculateDistance(BT::TreeNode& n) {
    geometry_msgs::msg::Vector3 p1, p2;
    p1.x = n.getInput<double>("x1").value();
    p1.y = n.getInput<double>("y1").value();
    p1.z = n.getInput<double>("z1").value();
    p2.x = n.getInput<double>("x2").value();
    p2.y = n.getInput<double>("y2").value();
    p2.z = n.getInput<double>("z2").value();

    n.setOutput<double>("dist", distance(p1, p2));
    return NodeStatus::SUCCESS;
}

/**
 * @brief Adds, subtracts, mutliplies, or divides two numbers and returns the result to the appropriate port.
 * 
 * @param n The BehaviorTree node.
 * @return BT::NodeStatus return status
 */
BT::NodeStatus doMath(BT::TreeNode& n) {
    double
        a = n.getInput<double>("a").value(),
        b = n.getInput<double>("b").value(),
        output = 0;
    
    std::string op = n.getInput<std::string>("operator").value();

    if(op == "+") {
        output = a + b;
    } else if(op == "-") {
        output = a - b;
    } else if(op == "*") {
        output = a * b;
    } else if(op == "/") {
        output = a / b;
    }

    n.setOutput<double>("out", output);
    return NodeStatus::SUCCESS;
}

/**
 * @brief Get the Heading To Point
 * 
 * @param n The BehaviorTree node.
 * @return BT::NodeStatus return status
 */
BT::NodeStatus getHeadingToPoint(BT::TreeNode& n) {
    double
        currX = n.getInput<double>("currX").value(),
        currY = n.getInput<double>("currY").value(),
        targX = n.getInput<double>("targX").value(),
        targY = n.getInput<double>("targY").value();

    double
        dx = targX - currX,
        dy = targY - currY;

    double heading = atan2(dy, dx);

    n.setOutput<double>("heading", heading);
    return NodeStatus::SUCCESS;
}

/**
 * @brief Registers simple actions to be done by the BehaviorTree.
 * 
 * @param factory The factory to register with.
 */
void SimpleActions::registerActions(BT::BehaviorTreeFactory *factory) {
    factory->registerSimpleAction("Info", printInfo, { InputPort<std::string>("message") } );
    factory->registerSimpleAction("Error", printError, { InputPort<std::string>("message") } );

    factory->registerSimpleAction("ToString", numToString,
        { 
            InputPort<double>("double_in"), 
            InputPort<int>("int_in"), 
            OutputPort<std::string>("str_out") 
        }
    );

    /**
     * Basic action that calculates the distance between two points.
     */
    factory->registerSimpleAction("CalculateDistance", calculateDistance,
        {
            InputPort<double>("x1"),
            InputPort<double>("y1"),
            InputPort<double>("z1"),
            InputPort<double>("x2"),
            InputPort<double>("y2"),
            InputPort<double>("z2"),
            OutputPort<double>("dist")
        }
    );

    /**
     * Basic action that does math with two numbers (can add, subtract, multiply, or divide)
     */
    factory->registerSimpleAction("Math", doMath, 
        {
            InputPort<double>("a"),
            InputPort<double>("b"),
            InputPort<std::string>("operator"),
            OutputPort<double>("out")
        }
    );

    factory->registerSimpleAction("HeadingToPoint", getHeadingToPoint,
        {
            InputPort<double>("currX"),
            InputPort<double>("currY"),
            InputPort<double>("targX"),
            InputPort<double>("targY"),
            OutputPort<double>("heading")
        }
    );
}
