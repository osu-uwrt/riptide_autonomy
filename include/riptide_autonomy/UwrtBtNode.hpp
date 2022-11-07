#pragma once

#include <rclcpp/rclcpp.hpp>

class UwrtBtNode {
    public:
    void init(rclcpp::Node::SharedPtr node) {
        this->rosnode = node;
        rosInit();
    }

    virtual void rosInit() = 0;

    protected:
    rclcpp::Node::SharedPtr rosnode;
};
