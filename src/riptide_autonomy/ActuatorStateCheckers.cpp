#include "autonomy.h"

/**
 * SIMPLE CONDITIONS FOR UWRT BEHAVIOR TREE
 */

/**
 * @brief Returns NodeStatus::SUCCESS if the value on the specified port equals the specified value.
 * 
 * @param node The TreeNode that contains the port to check.
 * @param portName The name of the port to check
 * @param value The value to check for 
 * @return BT::NodeStatus SUCCESS if the value on the port is equal to the value, FAILURE otherwise.
 */
BT::NodeStatus inputEquals(BT::TreeNode& node, std::string portName, int value) {
    int portVal = node.getInput<int>(portName).value();
    return (portVal == value ? NodeStatus::SUCCESS : NodeStatus::FAILURE);
}

/**
 * @brief Registers conditions with the given factory.
 * 
 * @param factory the factory to register the conditions with
 */
void ActuatorStateCheckers::registerConditions(BT::BehaviorTreeFactory factory) {
    factory.registerSimpleCondition("IsClawUnknown", ActuatorStateCheckers::isClawUnknown, { InputPort<int>("claw_state") });
    factory.registerSimpleCondition("IsClawOpen", ActuatorStateCheckers::isClawOpen, { InputPort<int>("claw_state") });
    factory.registerSimpleCondition("IsClawClosed", ActuatorStateCheckers::isClawClosed, { InputPort<int>("claw_state") });

    factory.registerSimpleCondition("IsTorpedoCharged", ActuatorStateCheckers::isTorpedoCharged, { InputPort<int>("torpedo_state") });
    factory.registerSimpleCondition("IsTorpedoFired", ActuatorStateCheckers::isTorpedoFired, { InputPort<int>("torpedo_state") });

    factory.registerSimpleCondition("IsDropperReady", ActuatorStateCheckers::isDropperReady, { InputPort<int>("dropper_state") });
    factory.registerSimpleCondition("IsDropperDropped", ActuatorStateCheckers::isDropperDropped, { InputPort<int>("dropper_state") });
}

BT::NodeStatus ActuatorStateCheckers::isClawUnknown(BT::TreeNode& node) {
    return inputEquals(node, "claw_state", riptide_msgs2::msg::ActuatorStatus::CLAW_UNKNOWN);
}

BT::NodeStatus ActuatorStateCheckers::isClawOpen(BT::TreeNode& node) {
    return inputEquals(node, "claw_state", riptide_msgs2::msg::ActuatorStatus::CLAW_OPENED);
}

BT::NodeStatus ActuatorStateCheckers::isClawClosed(BT::TreeNode& node) {
    return inputEquals(node, "claw_state", riptide_msgs2::msg::ActuatorStatus::CLAW_CLOSED);
}

BT::NodeStatus ActuatorStateCheckers::isTorpedoCharged(BT::TreeNode& node) {
    return inputEquals(node, "torpedo_state", riptide_msgs2::msg::ActuatorStatus::TORPEDO_CHARGED);
}

BT::NodeStatus ActuatorStateCheckers::isTorpedoFired(BT::TreeNode& node) {
    return inputEquals(node, "torpedo_state", riptide_msgs2::msg::ActuatorStatus::TORPEDO_FIRED);
}

BT::NodeStatus ActuatorStateCheckers::isDropperReady(BT::TreeNode& node) {
    return inputEquals(node, "dropper_state", riptide_msgs2::msg::ActuatorStatus::DROPPER_READY);
}

BT::NodeStatus ActuatorStateCheckers::isDropperDropped(BT::TreeNode& node) {
    return inputEquals(node, "dropper_state", riptide_msgs2::msg::ActuatorStatus::DROPPER_DROPPED);
}
