#include "autonomy.h"


/**
 * @brief Returns NodeStatus::SUCCESS if the value on the specified port equals the specified value.
 * 
 * @param node The TreeNode that contains the port to check.
 * @param portName The name of the port to check
 * @param value The value to check for 
 * @return BT::NodeStatus SUCCESS if the value on the port is equal to the value, FAILURE otherwise.
 */
BT::NodeStatus intInputEquals(BT::TreeNode& node, std::string portName, int value) {
    int portVal = node.getInput<int>(portName).value();
    return (portVal == value ? NodeStatus::SUCCESS : NodeStatus::FAILURE);
}

BT::NodeStatus isClawError(BT::TreeNode& node) {
    return intInputEquals(node, "claw_state", riptide_msgs2::msg::ActuatorStatus::CLAW_ERROR);
}

BT::NodeStatus isClawUnknown(BT::TreeNode& node) {
    return intInputEquals(node, "claw_state", riptide_msgs2::msg::ActuatorStatus::CLAW_UNKNOWN);
}

BT::NodeStatus isClawOpen(BT::TreeNode& node) {
    return intInputEquals(node, "claw_state", riptide_msgs2::msg::ActuatorStatus::CLAW_OPENED);
}

BT::NodeStatus isClawClosed(BT::TreeNode& node) {
    return intInputEquals(node, "claw_state", riptide_msgs2::msg::ActuatorStatus::CLAW_CLOSED);
}

BT::NodeStatus isTorpedoError(BT::TreeNode& node) {
    return intInputEquals(node, "torpedo_state", riptide_msgs2::msg::ActuatorStatus::TORPEDO_ERROR);
}

BT::NodeStatus isTorpedoCharging(BT::TreeNode& node) {
    return intInputEquals(node, "torpedo_state", riptide_msgs2::msg::ActuatorStatus::TORPEDO_CHARGING);
}

BT::NodeStatus isTorpedoCharged(BT::TreeNode& node) {
    return intInputEquals(node, "torpedo_state", riptide_msgs2::msg::ActuatorStatus::TORPEDO_CHARGED);
}

BT::NodeStatus isTorpedoFired(BT::TreeNode& node) {
    return intInputEquals(node, "torpedo_state", riptide_msgs2::msg::ActuatorStatus::TORPEDO_FIRED);
}

BT::NodeStatus isTorpedoDisarmed(BT::TreeNode& node) {
    return intInputEquals(node, "torpedo_state", riptide_msgs2::msg::ActuatorStatus::TORPEDO_DISARMED);
}

BT::NodeStatus isDropperError(BT::TreeNode& node) {
    return intInputEquals(node, "dropper_state", riptide_msgs2::msg::ActuatorStatus::DROPPER_ERROR);
}

BT::NodeStatus isDropperReady(BT::TreeNode& node) {
    return intInputEquals(node, "dropper_state", riptide_msgs2::msg::ActuatorStatus::DROPPER_READY);
}

BT::NodeStatus isDropperDropped(BT::TreeNode& node) {
    return intInputEquals(node, "dropper_state", riptide_msgs2::msg::ActuatorStatus::DROPPER_DROPPED);
}

/**
 * @brief Registers conditions with the given factory.
 * 
 * @param factory the factory to register the conditions with
 */
void ActuatorConditions::registerConditions(BT::BehaviorTreeFactory *factory) {
    factory->registerSimpleCondition("IsClawError", isClawError, { InputPort<int>("claw_state") });
    factory->registerSimpleCondition("IsClawUnknown", isClawUnknown, { InputPort<int>("claw_state") });
    factory->registerSimpleCondition("IsClawOpen", isClawOpen, { InputPort<int>("claw_state") });
    factory->registerSimpleCondition("IsClawClosed", isClawClosed, { InputPort<int>("claw_state") });

    factory->registerSimpleCondition("IsTorpedoError", isTorpedoError, { InputPort<int>("torpedo_state") });
    factory->registerSimpleCondition("IsTorpedoCharging", isTorpedoCharging, { InputPort<int>("torpedo_state") });
    factory->registerSimpleCondition("IsTorpedoCharged", isTorpedoCharged, { InputPort<int>("torpedo_state") });
    factory->registerSimpleCondition("IsTorpedoFired", isTorpedoFired, { InputPort<int>("torpedo_state") });
    factory->registerSimpleCondition("IsTorpedoDisarmed", isTorpedoDisarmed, { InputPort<int>("torpedo_state") });

    factory->registerSimpleCondition("IsDropperError", isDropperError, { InputPort<int>("dropper_state") });
    factory->registerSimpleCondition("IsDropperReady", isDropperReady, { InputPort<int>("dropper_state") });
    factory->registerSimpleCondition("IsDropperDropped", isDropperDropped, { InputPort<int>("dropper_state") });
}