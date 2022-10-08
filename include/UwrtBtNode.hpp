#pragma once

#include <rclcpp/rclcpp.hpp>

class UwrtBtNode {
    public:
    void init(rclcpp::Node::SharedPtr node) {
        this->rosnode = node;
    }

    protected:
    rclcpp::Node::SharedPtr rosnode;
};
