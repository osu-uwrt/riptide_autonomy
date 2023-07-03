#pragma once

#include "riptide_autonomy/autonomy_base.hpp"

class UwrtBtNode {
    public:
    void init(rclcpp::Node::SharedPtr node) {
        this->rosnode = node;
        rosInit();
    }

    const rclcpp::Node::SharedPtr rosNode() const {
        return rosnode;
    }

    virtual const BT::TreeNode *treeNode() const = 0;

    protected:
    virtual void rosInit() = 0;
    rclcpp::Node::SharedPtr rosnode;    
};

/**
 * @brief UWRT superclass for BT action nodes
 */
class UWRTActionNode : public BT::StatefulActionNode, public UwrtBtNode {
    public:
    UWRTActionNode(const std::string& name, const BT::NodeConfiguration& config)
     : StatefulActionNode(name, config) { };
    
    const BT::TreeNode *treeNode() const override {
        return this;
    }
};

/**
 * @brief UWRT superclass for integrating ConditionNodes with ROS.
 * Similar to UWRTSyncActionNode, this class inherits both the BT 
 * ConditionNode and UwrtBtNode.
 */
class UWRTConditionNode : public BT::ConditionNode, public UwrtBtNode {
    public:
    UWRTConditionNode(const std::string& name, const BT::NodeConfiguration& config)
     : ConditionNode(name, config) { };

    const BT::TreeNode *treeNode() const override {
        return this;
    }
};

/**
 * @brief UWRT superclass for integrating DecoratorNodes with ROS.
 * Operates exactly the same as UWRTConditionNode and UWRTActionNode.
 */
class UWRTDecoratorNode : public BT::DecoratorNode, public UwrtBtNode {
    public:
    UWRTDecoratorNode(const std::string& name, const BT::NodeConfiguration& config)
     : DecoratorNode(name, config) { };
    
    const BT::TreeNode *treeNode() const override {
        return this;
    }
};
