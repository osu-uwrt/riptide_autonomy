#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

#include <unordered_map>

/**
 * @brief SUCCESS if `object_name` was detected (at or above `min_score`) within the
 * last `max_age_secs`. Unlike WaitForDetection it answers immediately, so it can
 * guard a running move in a ReactiveSequence and interrupt it the moment the
 * object comes into view.
 */
class SeenRecently : public UWRTConditionNode {
    public:
    SeenRecently(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTConditionNode(name, config) {

    }

    static BT::PortsList providedPorts() {
        return {
            UwrtInput("object_name"),
            UwrtInput("max_age_secs"),
            UwrtInput("min_score") // mapping's confidence_cutoff is 0.2
        };
    }

    void rosInit() override {
        sub = rosNode()->create_subscription<vision_msgs::msg::Detection3DArray>(
            DETECTIONS_TOPIC,
            10,
            std::bind(&SeenRecently::detection3dArrayCb, this, _1));
    }

    BT::NodeStatus tick() override {
        const std::string object = tryGetRequiredInput<std::string>(this, "object_name", "");
        const double max_age = tryGetRequiredInput<double>(this, "max_age_secs", 0.5);
        const auto found = lastSeen.find(object);
        if(found == lastSeen.end()) {
            return BT::NodeStatus::FAILURE;
        }
        const double age = (rosNode()->get_clock()->now() - found->second).seconds();
        return (age >= 0 && age <= max_age) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    private:
    void detection3dArrayCb(const vision_msgs::msg::Detection3DArray::SharedPtr msg) {
        const double minScore = tryGetOptionalInput<double>(this, "min_score", 0.2);
        for(const auto& detection : msg->detections) {
            for(const auto& result : detection.results) {
                if(result.hypothesis.score >= minScore) {
                    lastSeen[result.hypothesis.class_id] = rosNode()->get_clock()->now();
                }
            }
        }
    }

    std::unordered_map<std::string, rclcpp::Time> lastSeen;
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr sub;
};
