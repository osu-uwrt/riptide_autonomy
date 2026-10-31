#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

/**
 * @brief BehaviorTree.CPP 3.8.7's Repeat, for the trees converted from v3 (bt3_to_bt4.py writes every Repeat as
 * RepeatV3). It ticks its child again within the same tick after each success, as v3 did; v4's Repeat returns
 * RUNNING between iterations instead, which re-ticks reactive parents in between.
 *
 * num_cycles: repeat a successful child N times (-1: forever). A failure ends it with FAILURE.
 */
class RepeatV3 : public BT::DecoratorNode {
    public:
    RepeatV3(const std::string& name, const BT::NodeConfiguration& config)
    : DecoratorNode(name, config) { }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("num_cycles", "Repeat a successful child N times (-1: forever)")
        };
    }

    BT::NodeStatus tick() override {
        const auto text = getInput<std::string>("num_cycles");
        try {
            if(!text) {
                throw std::invalid_argument(text.error());
            }
            numCycles = std::stoi(text.value()); // as v3 parsed it: "3.7" is 3
        } catch(const std::exception&) {
            throw BT::RuntimeError("Missing parameter [num_cycles] in RepeatNode");
        }

        setStatus(BT::NodeStatus::RUNNING);
        while(count < numCycles || numCycles == -1) {
            const BT::NodeStatus childStatus = child_node_->executeTick();
            switch(childStatus) {
                case BT::NodeStatus::SUCCESS:
                    count++;
                    haltChild();
                    break;
                case BT::NodeStatus::FAILURE:
                    count = 0;
                    haltChild();
                    return BT::NodeStatus::FAILURE;
                case BT::NodeStatus::RUNNING:
                    return BT::NodeStatus::RUNNING;
                case BT::NodeStatus::SKIPPED:
                    haltChild();
                    return BT::NodeStatus::SKIPPED;
                default:
                    throw BT::LogicError("A child node must never return IDLE");
            }
        }

        count = 0;
        return BT::NodeStatus::SUCCESS;
    }

    void halt() override {
        count = 0;
        DecoratorNode::halt();
    }

    private:
    int numCycles = 0, count = 0;
};
