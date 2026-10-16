#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

/**
 * @brief BehaviorTree.CPP 3.8.7's RetryUntilSuccessful, for the trees converted from v3 (bt3_to_bt4.py writes
 * every RetryUntilSuccessful as RetryUntilSuccessfulV3). It retries a failed child within the same tick, as v3 did;
 * v4's returns RUNNING between attempts instead.
 *
 * num_attempts: retry a failing child up to N times (-1: forever). A success ends it with SUCCESS.
 */
class RetryUntilSuccessfulV3 : public BT::DecoratorNode {
    public:
    RetryUntilSuccessfulV3(const std::string& name, const BT::NodeConfiguration& config)
    : DecoratorNode(name, config) { }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("num_attempts", "Retry a failing child up to N times (-1: forever)")
        };
    }

    BT::NodeStatus tick() override {
        const auto text = getInput<std::string>("num_attempts");
        try {
            if(!text) {
                throw std::invalid_argument(text.error());
            }
            maxAttempts = std::stoi(text.value()); // as v3 parsed it: "3.7" is 3
        } catch(const std::exception&) {
            throw BT::RuntimeError("Missing parameter [num_attempts] in RetryNode");
        }

        setStatus(BT::NodeStatus::RUNNING);
        while(attempt < maxAttempts || maxAttempts == -1) {
            const BT::NodeStatus childStatus = child_node_->executeTick();
            switch(childStatus) {
                case BT::NodeStatus::SUCCESS:
                    attempt = 0;
                    haltChild();
                    return BT::NodeStatus::SUCCESS;
                case BT::NodeStatus::FAILURE:
                    attempt++;
                    haltChild();
                    break;
                case BT::NodeStatus::RUNNING:
                    return BT::NodeStatus::RUNNING;
                case BT::NodeStatus::SKIPPED:
                    haltChild();
                    return BT::NodeStatus::SKIPPED;
                default:
                    throw BT::LogicError("A child node must never return IDLE");
            }
        }

        attempt = 0;
        return BT::NodeStatus::FAILURE;
    }

    void halt() override {
        attempt = 0;
        DecoratorNode::halt();
    }

    private:
    int maxAttempts = 0, attempt = 0;
};
