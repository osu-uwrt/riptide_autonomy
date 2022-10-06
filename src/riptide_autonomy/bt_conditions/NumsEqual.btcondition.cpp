#include "bt_conditions/NumsEqual.h"

using namespace BT;

BT::PortsList NumsEqual::providedPorts()
{
    return {
        BT::InputPort<double>("a"),
        BT::InputPort<double>("b")};
}

NodeStatus NumsEqual::tick()
{
    double
        a = getInput<double>("a").value(),
        b = getInput<double>("b").value();

    RCLCPP_WARN(log, "Using deprecated state NumsEqual! Use CompareNums instead.");

    return (a == b ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE);
}
