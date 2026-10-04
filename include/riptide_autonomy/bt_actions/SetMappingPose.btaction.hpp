#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"
#include "riptide_msgs2/srv/set_object_pose.hpp"
#include <sstream>

class SetMappingPose : public UWRTActionNode {
    using Service = riptide_msgs2::srv::SetObjectPose;

public:
    SetMappingPose(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config), result(std::future<Service::Response::SharedPtr>(), 0) { }

    static BT::PortsList providedPorts() {
        return {
            UwrtInput("object_name"), UwrtInput("reference_frame"),
            UwrtInput("x"), UwrtInput("y"), UwrtInput("z"),
            BT::InputPort<std::string>("or", "0", "Roll in radians"),
            BT::InputPort<std::string>("op", "0", "Pitch in radians"),
            BT::InputPort<std::string>("oy", "0", "Yaw in radians"),
            BT::InputPort<std::string>("update_orientation", "false", "Apply the supplied orientation"),
            BT::InputPort<std::string>("move_map", "false", "Translate the shared map offset"),
            BT::InputPort<std::string>("preserve_world_objects", "", "Semicolon-separated object names"),
            BT::InputPort<std::string>("srv_name", "mapping/set_object_pose", "Mapping service"),
            BT::InputPort<std::string>("time_limit_secs", "3", "Service timeout")
        };
    }

    void rosInit() override { }

    BT::NodeStatus onStart() override {
        client = rosnode->create_client<Service>(tryGetRequiredInput<std::string>(this, "srv_name", "mapping/set_object_pose"));
        timeoutSecs = tryGetRequiredInput<double>(this, "time_limit_secs", 3);
        startTime = rosnode->get_clock()->now();
        sent = false;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (!sent && client->service_is_ready()) {
            auto request = std::make_shared<Service::Request>();
            request->object_name = tryGetRequiredInput<std::string>(this, "object_name", "");
            request->pose.header.frame_id = tryGetRequiredInput<std::string>(this, "reference_frame", "");
            request->pose.pose.position.x = tryGetRequiredInput<double>(this, "x", 0);
            request->pose.pose.position.y = tryGetRequiredInput<double>(this, "y", 0);
            request->pose.pose.position.z = tryGetRequiredInput<double>(this, "z", 0);
            geometry_msgs::msg::Vector3 rpy;
            rpy.x = tryGetRequiredInput<double>(this, "or", 0);
            rpy.y = tryGetRequiredInput<double>(this, "op", 0);
            rpy.z = tryGetRequiredInput<double>(this, "oy", 0);
            request->pose.pose.orientation = toQuat(rpy);
            request->update_orientation = tryGetRequiredInput<bool>(this, "update_orientation", false);
            request->move_map = tryGetRequiredInput<bool>(this, "move_map", false);
            std::istringstream names(tryGetRequiredInput<std::string>(this, "preserve_world_objects", ""));
            std::string name;
            while (std::getline(names, name, ';')) {
                if (!name.empty()) request->preserve_world_objects.push_back(name);
            }
            result = client->async_send_request(request);
            sent = true;
        }
        if (sent && result.wait_for(0s) == std::future_status::ready) {
            const auto response = result.get();
            sent = false;
            if (!response->success) {
                RCLCPP_ERROR(rosnode->get_logger(), "SetMappingPose: %s", response->message.c_str());
            }
            return response->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }
        if ((rosnode->get_clock()->now() - startTime).seconds() > timeoutSecs) {
            onHalted();
            RCLCPP_ERROR(rosnode->get_logger(), "SetMappingPose service timed out");
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
        if (sent) client->remove_pending_request(result);
        sent = false;
    }

private:
    rclcpp::Client<Service>::SharedPtr client;
    rclcpp::Client<Service>::FutureAndRequestId result;
    rclcpp::Time startTime;
    double timeoutSecs = 3;
    bool sent = false;
};
