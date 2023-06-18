#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

enum TreeStatus {
    UNDEFINED,
    WAITING,
    STARTING,
    MOVING,
    SEARCHING,
    ALIGNING,
    PERFORMING,
    SUCCESS,
    FAILURE,
    PANIC
};

const std::map<std::string, TreeStatus> STATUSES = {
    { "undefined",  TreeStatus::UNDEFINED  },
    { "waiting",    TreeStatus::WAITING    },
    { "starting",   TreeStatus::STARTING   },
    { "moving",     TreeStatus::MOVING     },
    { "searching",  TreeStatus::SEARCHING  },
    { "aligning",   TreeStatus::ALIGNING   },
    { "performing", TreeStatus::PERFORMING },
    { "success",    TreeStatus::SUCCESS    },
    { "failure",    TreeStatus::FAILURE    },
    { "panic",      TreeStatus::PANIC      }
};

class SetStatus : public UWRTActionNode {
    using LedCmd = riptide_msgs2::msg::LedCommand;

    public:
    SetStatus(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config) { }

    /**
     * @brief Declares ports needed by this node.
     * @return PortsList Needed ports.
     */
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("status")
        };
    }

    /**
     * @brief Initializes ROS peripherals such as publishers, subscribers, actions, services, etc.
     * Anything requiring the ROS node handle to construct should be initialized here. Do not do it in the 
     * constructor or you will be very sad
     */
    void rosInit() override { 
        pub = rosnode->create_publisher<riptide_msgs2::msg::LedCommand>(LED_COMMAND_TOPIC, 10);
    }

    /**
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override {
        //get ID of requested LED status
        std::string statusName = tryGetRequiredInput<std::string>(this, "status", "undefined");
        if(STATUSES.count(statusName) == 0) {
            RCLCPP_WARN(log, "No behavior status by the name \"%s\"!", statusName.c_str());
            statusName = "undefined";
        }

        TreeStatus status = STATUSES.at(statusName);
        
        //configure an LED command to send that will indicate the robot status
        riptide_msgs2::msg::LedCommand cmd;
        switch(status) {
            case TreeStatus::UNDEFINED:
                cmd.red = 255;
                cmd.green = 255;
                cmd.blue = 255;
                cmd.mode = LedCmd::MODE_SOLID;
                cmd.target = LedCmd::TARGET_ALL;
                break;
            case TreeStatus::WAITING:
                cmd.red = 255;
                cmd.green = 0;
                cmd.blue = 0;
                cmd.mode = LedCmd::MODE_SOLID;
                cmd.target = LedCmd::TARGET_ALL;
                break;
            case TreeStatus::STARTING:
                cmd.red = 0;
                cmd.green = 0;
                cmd.blue = 255;
                cmd.mode = LedCmd::MODE_SLOW_FLASH;
                cmd.target = LedCmd::TARGET_ALL;
                break;
            case TreeStatus::MOVING:
                cmd.red = 0;
                cmd.green = 255;
                cmd.blue = 0;
                cmd.mode = LedCmd::MODE_SOLID;
                cmd.target = LedCmd::TARGET_ALL;
                break;
            case TreeStatus::SEARCHING:
                cmd.red = 255;
                cmd.green = 255;
                cmd.blue = 0;
                cmd.mode = LedCmd::MODE_SOLID;
                cmd.target = LedCmd::TARGET_ALL;
                break;
            case TreeStatus::ALIGNING:
                cmd.red = 255;
                cmd.green = 0;
                cmd.blue = 255;
                cmd.mode = LedCmd::MODE_SLOW_FLASH;
                cmd.target = LedCmd::TARGET_ALL;
                break;
            case TreeStatus::PERFORMING:
                cmd.red = 0;
                cmd.green = 0;
                cmd.blue = 255;
                cmd.mode = LedCmd::MODE_FAST_FLASH;
                cmd.target = LedCmd::TARGET_ALL;
                break;
            case TreeStatus::SUCCESS:
                cmd.red = 0;
                cmd.green = 255;
                cmd.blue = 0;
                cmd.mode = LedCmd::MODE_BREATH;
                cmd.target = LedCmd::TARGET_ALL;
                break;
            case TreeStatus::FAILURE:
                cmd.red = 255;
                cmd.green = 0;
                cmd.blue = 0;
                cmd.mode = LedCmd::MODE_BREATH;
                cmd.target = LedCmd::TARGET_ALL;
                break;
            case TreeStatus::PANIC:
                cmd.red = 255;
                cmd.green = 0;
                cmd.blue = 0;
                cmd.mode = LedCmd::MODE_FAST_FLASH;
                cmd.target = LedCmd::TARGET_ALL;
                break;
        }

        pub->publish(cmd);
        return BT::NodeStatus::SUCCESS;
    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after 
     */
    BT::NodeStatus onRunning() override {
        return BT::NodeStatus::SUCCESS;
    }

    /**
     * @brief Called when the node is halted.
     */
    void onHalted() override {

    }

    private:
    rclcpp::Publisher<riptide_msgs2::msg::LedCommand>::SharedPtr pub;
};
