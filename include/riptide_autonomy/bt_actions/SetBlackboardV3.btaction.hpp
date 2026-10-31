#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

/**
 * @brief BehaviorTree.CPP 3.8.7's SetBlackboard, for the trees converted from v3 (bt3_to_bt4.py writes every
 * SetBlackboard as SetBlackboardV3). v4's own SetBlackboard differs: it fails when copying an empty entry and can
 * leave a key empty that another SetBlackboard then copies from.
 *
 * output_key is the key to write (a key name, or {key} holding the name). value is stored as text, or, as {key},
 * that entry is copied (its value and type) into output_key.
 */
class SetBlackboardV3 : public BT::SyncActionNode {
    public:
    SetBlackboardV3(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config) { }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("value", "Text to store, or {key} to copy that entry"),
            BT::BidirectionalPort<std::string>("output_key", "Key to write (a key name, or {key} holding the key name)")
        };
    }

    BT::NodeStatus tick() override {
        std::string outputKey;
        if(!getInput("output_key", outputKey)) {
            throw BT::RuntimeError("missing port [output_key]");
        }

        const auto &blackboard = config().blackboard;
        const std::string valueText = config().input_ports.at("value");
        BT::StringView stripped;
        if(isBlackboardPointer(valueText, &stripped)) {
            const auto source = blackboard->getEntry(std::string(stripped));
            if(!source) {
                throw BT::RuntimeError("Can't find the port referred by [value]");
            }
            auto destination = blackboard->getEntry(outputKey);
            if(!destination) {
                blackboard->createEntry(outputKey, source->info);
                destination = blackboard->getEntry(outputKey);
            }
            if(destination != source) {
                std::scoped_lock locks(source->entry_mutex, destination->entry_mutex);
                destination->value = source->value;
                destination->sequence_id++;
                destination->stamp = std::chrono::steady_clock::now().time_since_epoch();
            }
        } else {
            blackboard->set(outputKey, valueText);
        }

        return BT::NodeStatus::SUCCESS;
    }
};
