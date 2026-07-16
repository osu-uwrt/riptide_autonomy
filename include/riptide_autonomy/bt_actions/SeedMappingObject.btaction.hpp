#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

class SeedMappingObject : public UWRTActionNode {
    public:
    using SeedObjectPose = riptide_msgs2::srv::SeedObjectPose;

    SeedMappingObject(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config),
      result(std::future<std::shared_ptr<SeedObjectPose::Response>>(), 0) {

    }

    /**
     * @brief Declares ports needed by this node.
     * @return PortsList Needed ports.
     */
    static BT::PortsList providedPorts() {
        return {
            UwrtInput("object_name"),
            UwrtInput("x"),
            UwrtInput("y"),
            UwrtInput("z"),
            UwrtInput("time_limit_secs")
        };
    }

    /**
     * @brief Initializes ROS peripherals such as publishers, subscribers, actions, services, etc.
     * Anything requiring the ROS node handle to construct should be initialized here. Do not do it in the
     * constructor or you will be very sad
     */
    void rosInit() override {
        client = rosNode()->create_client<SeedObjectPose>(MAPPING_SEED_POSE_SERVER_NAME);
    }

    /**
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override {
        timeoutSecs = tryGetRequiredInput<double>(this, "time_limit_secs", 3);

        //is client available?
        if(!client->wait_for_service(1s)) {
            RCLCPP_ERROR(rosNode()->get_logger(), "Mapping service %s not available.", MAPPING_SEED_POSE_SERVER_NAME.c_str());
            return BT::NodeStatus::FAILURE;
        }

        //get node arguments. position is the desired object position in the map frame
        std::string object = tryGetRequiredInput<std::string>(this, "object_name", "");
        double
            x = tryGetRequiredInput<double>(this, "x", 0),
            y = tryGetRequiredInput<double>(this, "y", 0),
            z = tryGetRequiredInput<double>(this, "z", 0);

        RCLCPP_INFO(rosNode()->get_logger(), "Seeding mapping object %s to map position %f, %f, %f", object.c_str(), x, y, z);

        //make request
        auto request = std::make_shared<SeedObjectPose::Request>();
        request->object_name = object;
        request->position.x = x;
        request->position.y = y;
        request->position.z = z;

        //make call
        result = client->async_send_request(request);
        startTime = rosNode()->get_clock()->now();
        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after
     */
    BT::NodeStatus onRunning() override {
        if(!result.valid()) {
            RCLCPP_ERROR(rosNode()->get_logger(), "Result of SeedObjectPose service call to %s is not valid.", MAPPING_SEED_POSE_SERVER_NAME.c_str());
            return BT::NodeStatus::FAILURE;
        }

        if(result.wait_for(0s) == std::future_status::ready) {
            auto response = result.get();
            if(!response->success) {
                RCLCPP_ERROR(rosNode()->get_logger(), "Failed to seed mapping object: %s", response->message.c_str());
                return BT::NodeStatus::FAILURE;
            }

            return BT::NodeStatus::SUCCESS;
        }

        //not ready, check for timeout
        if((rosNode()->get_clock()->now() - startTime).seconds() > timeoutSecs) {
            RCLCPP_ERROR(rosNode()->get_logger(), "Service call to %s timed out.", MAPPING_SEED_POSE_SERVER_NAME.c_str());
            return BT::NodeStatus::FAILURE;
        }

        //we'll get em next time
        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called when the node is halted.
     */
    void onHalted() override {

    }

    private:
    double timeoutSecs;
    rclcpp::Time startTime;
    rclcpp::Client<SeedObjectPose>::SharedPtr client;
    rclcpp::Client<SeedObjectPose>::FutureAndRequestId result;
};
