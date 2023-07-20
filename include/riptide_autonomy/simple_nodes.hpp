#pragma once

#include "autonomy_lib.hpp"

class UWRTSimpleActionNode : public BT::SyncActionNode, public UwrtBtNode {
    public:
    typedef std::function<BT::NodeStatus(UwrtBtNode&)> TickFunctor;

    UWRTSimpleActionNode(const std::string& name, const TickFunctor& tickFunctor, const BT::NodeConfiguration& config)
     : SyncActionNode(name, config),
       func(tickFunctor) { };
    
    BT::TreeNode *treeNode() override {
        return this;
    }

    protected:
    BT::NodeStatus tick() override {
        BT::NodeStatus prevStatus = status();
        if(prevStatus == BT::NodeStatus::IDLE) {
            setStatus(BT::NodeStatus::RUNNING);
            prevStatus = BT::NodeStatus::RUNNING;
        }

        BT::NodeStatus newStatus = func(*this);
        if(newStatus != prevStatus) {
            setStatus(newStatus);
        }

        return newStatus;
    }

    void rosInit() override { }

    private:
    TickFunctor func;
};

void registerSimpleUwrtAction(BT::BehaviorTreeFactory& factory, const std::string& id, const UWRTSimpleActionNode::TickFunctor& tickFunctor, BT::PortsList ports);
void bulkRegisterSimpleActions(BT::BehaviorTreeFactory& factory);
