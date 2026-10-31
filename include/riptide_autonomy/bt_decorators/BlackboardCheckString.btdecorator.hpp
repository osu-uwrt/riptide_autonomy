#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

/**
 * @brief BehaviorTree.CPP 3.8.7's BlackboardCheckString (removed in v4), kept under its own name for the trees
 * converted from v3.
 *
 * Every tick: when value_A and value_B read as equal text, the child is ticked and its status returned; otherwise
 * a running child is halted and return_on_mismatch (IDLE, RUNNING, SUCCESS or FAILURE) is returned. Anything else
 * there, or a value that cannot be read, gives FAILURE, as in v3 (so the trees' return_on_mismatch="0" fails).
 */
class BlackboardCheckString : public BT::DecoratorNode {
    public:
    BlackboardCheckString(const std::string& name, const BT::NodeConfiguration& config)
    : DecoratorNode(name, config) { }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("value_A"),
            BT::InputPort<std::string>("value_B"),
            BT::InputPort<std::string>("return_on_mismatch", "Status when value_A != value_B")
        };
    }

    BT::NodeStatus tick() override {
        setStatus(BT::NodeStatus::RUNNING);

        const auto a = getInput<std::string>("value_A");
        const auto b = getInput<std::string>("value_B");
        if(a && b && a.value() == b.value()) {
            return child_node_->executeTick();
        }

        if(child()->status() == BT::NodeStatus::RUNNING) {
            haltChild();
        }

        const auto onMismatch = getInput<std::string>("return_on_mismatch");
        if(onMismatch) {
            for(const auto status : {BT::NodeStatus::IDLE, BT::NodeStatus::RUNNING, BT::NodeStatus::SUCCESS,
                                     BT::NodeStatus::FAILURE}) {
                if(onMismatch.value() == BT::toStr(status)) {
                    return status;
                }
            }
        }
        return BT::NodeStatus::FAILURE;
    }
};
