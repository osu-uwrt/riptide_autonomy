#include "riptide_autonomy/simple_nodes.hpp"


/**
 * @brief Prints a string to the rclcpp info console.
 * 
 * @param n The BehaviorTree node.
 * @return BT::NodeStatus return status.
 */
BT::NodeStatus printInfo(UwrtBtNode& n) {
    std::string message = tryGetRequiredInput<std::string>(&n, "message", "");
    RCLCPP_INFO(n.rosNode()->get_logger(), "%s", formatStringWithBlackboard(message, &n).c_str()); 
    return BT::NodeStatus::SUCCESS; 
}

/**
 * @brief Prints a string to the rclcpp error console.
 * 
 * @param n The BehaviorTree node.
 * @return BT::NodeStatus return status.
 */
BT::NodeStatus printError(UwrtBtNode& n) {
    std::string message = tryGetRequiredInput<std::string>(&n, "message", "");
    RCLCPP_ERROR(n.rosNode()->get_logger(), "%s", formatStringWithBlackboard(message, &n).c_str());
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief Calculates the distance between two 3d points and returns it to the appropriate port.
 * 
 * @param n The BehaviorTree node.
 * @return BT::NodeStatus return status
 */
BT::NodeStatus calculateDistance(UwrtBtNode& n) {
    geometry_msgs::msg::Vector3 p1, p2;
    p1.x = tryGetRequiredInput<double>(&n, "x1", 0);
    p1.y = tryGetRequiredInput<double>(&n, "y1", 0);
    p1.z = tryGetRequiredInput<double>(&n, "z1", 0);
    p2.x = tryGetRequiredInput<double>(&n, "x2", 0);
    p2.y = tryGetRequiredInput<double>(&n, "y2", 0);
    p2.z = tryGetRequiredInput<double>(&n, "z2", 0);

    postOutput<double>(&n, "dist", distance(p1, p2));
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief Adds, subtracts, mutliplies, or divides two numbers and returns the result to the appropriate port.
 * 
 * @param n The BehaviorTree node.
 * @return BT::NodeStatus return status
 */
BT::NodeStatus doMath(UwrtBtNode& n) {
    double
        a = tryGetRequiredInput<double>(&n, "a", 0),
        b = tryGetRequiredInput<double>(&n, "b", 0),
        output = 0;
    
    std::string op = tryGetRequiredInput<std::string>(&n, "operator", "+");

    if(op == "+") {
        output = a + b;
    } else if(op == "-") {
        output = a - b;
    } else if(op == "*") {
        output = a * b;
    } else if(op == "/") {
        output = a / b;
    }

    postOutput<double>(&n, "out", output);
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief Formats a string for BT
 * @param n Tree node
 * @return execution status
 */
BT::NodeStatus format(UwrtBtNode &n) {
    std::string formatStr = tryGetRequiredInput<std::string>(&n, "format", "");
    std::string out = formatStringWithBlackboard(formatStr, &n);
    postOutput<std::string>(&n, "out", out);
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief Get the Heading To Point
 * 
 * @param n The BehaviorTree node.
 * @return BT::NodeStatus return status
 */
BT::NodeStatus getHeadingToPoint(UwrtBtNode& n) {
    double
        currX = tryGetRequiredInput<double>(&n, "currX", 0),
        currY = tryGetRequiredInput<double>(&n, "currY", 0),
        targX = tryGetRequiredInput<double>(&n, "targX", 0),
        targY = tryGetRequiredInput<double>(&n, "targY", 0);

    double
        dx = targX - currX,
        dy = targY - currY;

    double heading = atan2(dy, dx);

    postOutput<double>(&n, "heading", heading);
    return BT::NodeStatus::SUCCESS;
}


void registerSimpleUwrtAction(BT::BehaviorTreeFactory& factory, const std::string& id, const UWRTSimpleActionNode::TickFunctor& tickFunctor, BT::PortsList ports) {
    BT::NodeBuilder builder = [tickFunctor, id] (const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<UWRTSimpleActionNode>(name, tickFunctor, config);
    };

    BT::TreeNodeManifest manifest = {BT::NodeType::ACTION, id, std::move(ports), {}};
    factory.registerBuilder(manifest, builder);
}

/**
 * @brief Registers simple actions to be done by the BehaviorTree.
 * 
 * @param factory The factory to register with.
 */
void bulkRegisterSimpleActions(BT::BehaviorTreeFactory &factory) {
    registerSimpleUwrtAction(factory, "Info", printInfo, { BT::InputPort<std::string>("message") } );
    registerSimpleUwrtAction(factory, "Error", printError, { BT::InputPort<std::string>("message") } );

    /**
     * Basic action that calculates the distance between two points.
     */
    registerSimpleUwrtAction(factory, "CalculateDistance", calculateDistance,
        {
            UwrtInput("x1"),
            UwrtInput("y1"),
            UwrtInput("z1"),
            UwrtInput("x2"),
            UwrtInput("y2"),
            UwrtInput("z2"),
            UwrtOutput("dist")
        }
    );

    /**
     * Basic action that does math with two numbers (can add, subtract, multiply, or divide)
     */
    registerSimpleUwrtAction(factory, "Math", doMath, 
        {
            UwrtInput("a"),
            UwrtInput("b"),
            UwrtInput("operator"),
            UwrtOutput("out")
        }
    );

    registerSimpleUwrtAction(factory, "Format", format,
        {
            UwrtInput("format"),
            UwrtOutput("out")
        }
    );

    registerSimpleUwrtAction(factory, "HeadingToPoint", getHeadingToPoint,
        {
            UwrtInput("currX"),
            UwrtInput("currY"),
            UwrtInput("targX"),
            UwrtInput("targY"),
            UwrtOutput("heading")
        }
    );
}
