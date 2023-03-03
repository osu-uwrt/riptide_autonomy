#pragma once

#include "riptide_autonomy/autonomy_base.hpp"

class UwrtBtNode {
    public:
    void init(rclcpp::Node::SharedPtr node) {
        this->rosnode = node;
        rosInit();
    }

    protected:
    virtual void rosInit() = 0;    
    rclcpp::Node::SharedPtr rosnode;
};
