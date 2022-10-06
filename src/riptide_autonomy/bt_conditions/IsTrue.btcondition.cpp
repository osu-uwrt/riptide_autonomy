#include "bt_conditions/IsTrue.h"

using namespace BT;

BT::PortsList IsTrue::providedPorts()
{
    return {
        BT::InputPort<bool>("value")};
}

NodeStatus IsTrue::tick()
{
    bool value = getInput<bool>("value").value();
    return (value ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE);
}
