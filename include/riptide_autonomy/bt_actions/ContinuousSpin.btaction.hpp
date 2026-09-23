#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"
#include <algorithm>
#include <cmath>
#include <mutex>

// Ramp a level yaw target while counting actual, unwrapped odometry rotation.
class ContinuousSpin : public UWRTActionNode {
public:
    ContinuousSpin(const std::string& name, const BT::NodeConfiguration& config)
        : UWRTActionNode(name, config) { }

    static BT::PortsList providedPorts() {
        return {
            UwrtInput("num_cycles", "Number of full counterclockwise turns"),
            UwrtInput("yaw_rate", "Target ramp speed in radians/second; default 0.5"),
            UwrtInput("timeout_secs", "Overall timeout; default 90 seconds"),
            UwrtInput("odom_timeout_secs", "Odometry watchdog; default 1 second")
        };
    }

    void rosInit() override {
        linearPub = rosnode->create_publisher<riptide_msgs2::msg::ControllerCommand>(CONTROL_LINEAR_TOPIC, 10);
        angularPub = rosnode->create_publisher<riptide_msgs2::msg::ControllerCommand>(CONTROL_ANGULAR_TOPIC, 10);
        odomSub = rosnode->create_subscription<nav_msgs::msg::Odometry>(ODOMETRY_TOPIC, 10,
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex);
                if (!active) return;
                const auto& q = msg->pose.pose.orientation;
                const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
                if (!std::isfinite(norm) || norm < 1e-12) return;
                const auto rpy = toRPY(q);
                const auto& p = msg->pose.pose.position;
                if (!std::isfinite(rpy.x) || !std::isfinite(rpy.y) || !std::isfinite(rpy.z) ||
                    !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
                    !std::isfinite(msg->twist.twist.angular.z)) return;
                if (initialized) {
                    // Each odometry interval must contain less than half a turn.
                    travelled += std::remainder(rpy.z - lastYaw, TWO_PI);
                }
                lastYaw = rpy.z;
                odom = *msg;
                received = true;
                lastOdom = Clock::now();
            });
    }

    BT::NodeStatus onStart() override {
        std::lock_guard<std::mutex> lock(mutex);
        active = initialized = received = false;
        travelled = commanded = 0;
        const int cycles = tryGetRequiredInput<int>(this, "num_cycles", -1);
        rate = tryGetOptionalInput<double>(this, "yaw_rate", 0.5);
        timeout = tryGetOptionalInput<double>(this, "timeout_secs", 90);
        odomTimeout = tryGetOptionalInput<double>(this, "odom_timeout_secs", 1);
        if (cycles < 0 || !std::isfinite(rate) || rate <= 0 ||
            !std::isfinite(timeout) || timeout <= 0 ||
            !std::isfinite(odomTimeout) || odomTimeout <= 0) {
            RCLCPP_ERROR(rosnode->get_logger(), "ContinuousSpin requires nonnegative cycles and positive finite rate/timeouts.");
            return BT::NodeStatus::FAILURE;
        }
        if (cycles == 0) return BT::NodeStatus::SUCCESS;
        target = cycles * TWO_PI;
        started = lastOdom = Clock::now();
        active = true;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        std::lock_guard<std::mutex> lock(mutex);
        const auto now = Clock::now();
        if (seconds(now - started) > timeout || seconds(now - lastOdom) > odomTimeout) {
            RCLCPP_ERROR(rosnode->get_logger(), "ContinuousSpin timed out (rotation %.3f / %.3f rad).", travelled, target);
            holdCurrentHeading();
            active = false;
            return BT::NodeStatus::FAILURE;
        }
        if (!received) return BT::NodeStatus::RUNNING;
        if (!initialized) {
            initialYaw = lastYaw;
            position = odom.pose.pose.position;
            lastTick = now;
            initialized = true;
        }

        // A late tick or a stalled robot must not let the target lap the robot.
        const double dt = std::max(0.0, std::min(seconds(now - lastTick), 0.1));
        lastTick = now;
        commanded = std::max(0.0, std::min({target, commanded + rate * dt, travelled + MAX_LEAD}));
        publishPosition();
        publishHeading(initialYaw + commanded);

        const auto rpy = toRPY(odom.pose.pose.orientation);
        if (commanded >= target && std::abs(target - travelled) < 0.05 &&
            std::abs(odom.twist.twist.angular.z) < 0.05 &&
            std::abs(rpy.x) < 0.1 && std::abs(rpy.y) < 0.1) {
            active = false;
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
        std::lock_guard<std::mutex> lock(mutex);
        if (active) holdCurrentHeading();
        active = false;
    }

private:
    using Clock = std::chrono::steady_clock;
    static constexpr double TWO_PI = 6.283185307179586;
    static constexpr double MAX_LEAD = 0.6; // radians, well below the shortest-path ambiguity at pi
    static double seconds(Clock::duration duration) {
        return std::chrono::duration<double>(duration).count();
    }

    void publishPosition() {
        riptide_msgs2::msg::ControllerCommand cmd;
        cmd.mode = riptide_msgs2::msg::ControllerCommand::POSITION;
        cmd.setpoint_vect = pointToVector3(position);
        linearPub->publish(cmd);
    }

    void publishHeading(double yaw) {
        riptide_msgs2::msg::ControllerCommand cmd;
        cmd.mode = riptide_msgs2::msg::ControllerCommand::POSITION;
        cmd.setpoint_vect.z = std::remainder(yaw, TWO_PI);
        cmd.setpoint_quat = toQuat(cmd.setpoint_vect); // zero roll and pitch
        angularPub->publish(cmd);
    }

    void holdCurrentHeading() {
        if (initialized) {
            publishPosition();
            publishHeading(lastYaw);
        }
    }

    std::mutex mutex;
    bool active = false, initialized = false, received = false;
    double rate = 0, timeout = 0, odomTimeout = 0;
    double target = 0, travelled = 0, commanded = 0, initialYaw = 0, lastYaw = 0;
    Clock::time_point started, lastOdom, lastTick;
    nav_msgs::msg::Odometry odom;
    geometry_msgs::msg::Point position;
    rclcpp::Publisher<riptide_msgs2::msg::ControllerCommand>::SharedPtr linearPub, angularPub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;
};
