#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"

#include <cmath>
#include <optional>
#include <unordered_map>

/**
 * @brief SUCCESS once mapping's estimate of `object` is better than `target_cov`, the
 * same error getCovariance reports. It answers immediately, so it can guard a running
 * move in a ReactiveSequence (look around until the map has the object) and stop it
 * the moment the estimate converges. Only estimates published after the condition
 * started being ticked count, so one from before a SetMappingTarget reset can't pass it.
 */
class MapConverged : public UWRTConditionNode {
    using Cov = geometry_msgs::msg::PoseWithCovarianceStamped;

    public:
    MapConverged(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTConditionNode(name, config) {

    }

    static BT::PortsList providedPorts() {
        return {
            UwrtInput("object", "Mapping object (mapping/<object>)"),
            UwrtInput("target_cov", "Converged below this error (getCovariance's)")
        };
    }

    void rosInit() override {

    }

    BT::NodeStatus tick() override {
        const std::string object = tryGetRequiredInput<std::string>(this, "object", "");
        const double target = tryGetRequiredInput<double>(this, "target_cov", 0.0);
        const rclcpp::Time now = rosNode()->get_clock()->now();
        // Not ticked for a while: a new check, which only trusts estimates from now on
        if(!lastTick || (now - *lastTick).seconds() > 1.0 || object != checking) {
            since = now;
            checking = object;
        }
        lastTick = now;
        if(object.empty()) {
            return BT::NodeStatus::FAILURE;
        }
        subscribe(object);
        const auto found = latest.find(object);
        if(found == latest.end() || found->second.stamp < since) {
            return BT::NodeStatus::FAILURE;
        }
        return found->second.error < target ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    // getCovariance's error: root of the summed squared variances, the angular ones over 2 pi
    static double error(const Cov& msg) {
        double sum = 0;
        for(int i = 0; i < 3; i++) {
            sum += std::pow(msg.pose.covariance.at(6 * i + i), 2);
        }
        for(int i = 3; i < 6; i++) {
            sum += std::pow(msg.pose.covariance.at(6 * i + i), 2) / (2 * M_PI);
        }
        return std::sqrt(sum);
    }

    private:
    struct Estimate {
        rclcpp::Time stamp;
        double error;
    };

    void subscribe(const std::string& object) {
        if(subs.count(object)) {
            return;
        }
        subs[object] = rosNode()->create_subscription<Cov>("mapping/" + object, rclcpp::SensorDataQoS(),
            [this, object](const Cov::SharedPtr msg) {
                latest[object] = {rosNode()->get_clock()->now(), error(*msg)};
            });
    }

    std::unordered_map<std::string, rclcpp::Subscription<Cov>::SharedPtr> subs;
    std::unordered_map<std::string, Estimate> latest;
    std::optional<rclcpp::Time> lastTick;
    rclcpp::Time since;
    std::string checking;
};
