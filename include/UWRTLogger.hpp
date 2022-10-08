#pragma once

#include <string>
#include <vector>

#include <iostream>

#include <behaviortree_cpp_v3/loggers/abstract_logger.h>
#include <rclcpp/rclcpp.hpp>

#include <riptide_msgs2/msg/tree_stack.hpp>

class UwrtLogger : public BT::StatusChangeLogger
{
protected:
    static std::atomic<bool> ref_count;

public:
    UwrtLogger(const BT::Tree &tree, rclcpp::Node::SharedPtr node) : StatusChangeLogger(tree.rootNode())
    {
        bool expected = false;
        if (!ref_count.compare_exchange_strong(expected, true))
        {
            throw BT::LogicError("Only one instance of UwrtLogger shall be created");
        }

        stackPub = node->create_publisher<riptide_msgs2::msg::TreeStack>("autonomy/tree_stack", rclcpp::SystemDefaultsQoS());
        treeStack = std::vector<std::string>();
    }

    ~UwrtLogger() override
    {
        ref_count.store(false);
        flush();
    }

    virtual void callback(BT::Duration timestamp, const BT::TreeNode &node, BT::NodeStatus prev_status,
                          BT::NodeStatus status) override
    {

        std::cout << node.name().c_str() << " ";
        
        // nodes that take time
        if(prev_status != BT::NodeStatus::RUNNING && status == BT::NodeStatus::RUNNING){
            treeStack.emplace_back(node.name());
            sendStack();
            std::cout << "added to stack " << treeStack.size();
        }

        // cleanup for nodes that take time
        else if(prev_status == BT::NodeStatus::RUNNING && status != BT::NodeStatus::RUNNING){
            std::cout << "removed from stack " << treeStack.size();

            safePop();
            sendStack();
        }

        // nodes that complete immediately
        else if (prev_status == BT::NodeStatus::IDLE && status != BT::NodeStatus::RUNNING){
            treeStack.emplace_back(node.name());

            std::cout << "added and removed stack " << treeStack.size();
            sendStack();

            safePop();
        }

        std::cout << std::endl;
        
    }

    void flush() override
    {
        std::cout << "Flushing UWRT logger: stack size " << treeStack.size() << std::endl;
        // for(auto item : treeStack){
        //     std::cout << "'" << item << "'" << std::endl;
        // }
        sendStack();
    }

protected:
    void safePop(){
        if(treeStack.size() > 0 ){
            // we can cleanly pop
            treeStack.pop_back();
        } else{
            throw BT::RuntimeError("Attempted to pop a stack of size 0");
        }
    }

    void sendStack()
    {
        auto stackMsg = std::make_shared<riptide_msgs2::msg::TreeStack>();
        stackMsg->stack = treeStack;

        stackPub->publish(*stackMsg);
    }

    rclcpp::Publisher<riptide_msgs2::msg::TreeStack>::SharedPtr stackPub;
    std::vector<std::string> treeStack;
};

std::atomic<bool> UwrtLogger::ref_count(false);

//ros2 action send_goal /tempest/autonomy/run_tree riptide_msgs2/action/ExecuteTree "tree: '/home/coalman321/osu-uwrt/riptide_software/install/riptide_autonomy2/share/riptide_autonomy2/trees/ActuatorTrees.xml'"
