#include "bt_conditions/CompareNums.h"

using namespace BT;

BT::PortsList CompareNums::providedPorts()
{
    return {
        BT::InputPort<std::string>("test"),
        BT::InputPort<double>("a"),
        BT::InputPort<double>("b")};
}

NodeStatus CompareNums::tick()
{
    std::string test = getInput<std::string>("test").value();

    double
        a = getInput<double>("a").value(),
        b = getInput<double>("b").value();

    if (test == ">")
    { // check a > b
        return (a > b ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE);
    }
    else if (test == "<")
    { // check if a < b
        return (a < b ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE);
    }
    else if (test == "=")
    { // check if a == b
        return (a == b ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE);
    }

    RCLCPP_ERROR(log, "Invalid operator %s!", test.c_str());
    return BT::NodeStatus::FAILURE;
}
