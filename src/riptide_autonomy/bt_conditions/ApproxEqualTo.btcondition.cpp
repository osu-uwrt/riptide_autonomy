#include "bt_conditions/ApproxEqualTo.h"

using namespace BT;

BT::PortsList ApproxEqualTo::providedPorts()
{
    return {
        BT::InputPort<double>("a"),
        BT::InputPort<double>("b"),
        BT::InputPort<double>("range")};
}

NodeStatus ApproxEqualTo::tick()
{
    double
        a = getInput<double>("a").value(),
        b = getInput<double>("b").value(),
        range = getInput<double>("range").value();

    return (abs(a - b) < range ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE);
}
