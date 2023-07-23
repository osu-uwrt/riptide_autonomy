#pragma once

#include "riptide_autonomy/autonomy_base.hpp"

class UwrtBtNode {
    public:
    static void staticInit(rclcpp::Node::SharedPtr node);
    static void staticDeinit();
    void init(rclcpp::Node::SharedPtr node);

    const rclcpp::Node::SharedPtr rosNode() const {
        return rosnode;
    }

    virtual BT::TreeNode *treeNode() = 0;

    protected:
    virtual void rosInit() = 0;
    static std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    rclcpp::Node::SharedPtr rosnode;

    private:
    static std::shared_ptr<tf2_ros::TransformListener> tfListener;
};



/**
 * @brief UWRT superclass for BT action nodes
 */
class UWRTActionNode : public BT::StatefulActionNode, public UwrtBtNode {
    public:
    UWRTActionNode(const std::string& name, const BT::NodeConfiguration& config)
     : StatefulActionNode(name, config) { };
    
    BT::TreeNode *treeNode() override {
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

    BT::TreeNode *treeNode() override {
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
    
    BT::TreeNode *treeNode() override {
        return this;
    }
};
