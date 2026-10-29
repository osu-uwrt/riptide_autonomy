#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"
#include <algorithm>
#include <cmath>
#include <mutex>
#include <optional>

// Chase the latest detection of object_name with its (forward) camera: move onto the object's front (its mapping frame's
// +x side, at the range it was seen from), level with it, so the vehicle ends square on with its +x along the object's
// yaw and the detection centered, while yawing at the detection the whole way so it stays centered in view. Aims at
// each detection's world position (looked up at its own stamp, so turning doesn't smear it): a target first seen at
// the image edge, where its box is cut off and its center is off, is brought into the middle before mapping relies on
// it. align_yaw false: only yaw and depth, holding position. SUCCESS once there (the setpoints stay commanded),
// FAILURE if it is lost or the timeout passes.
class CenterDetection : public UWRTActionNode {
public:
    CenterDetection(const std::string& name, const BT::NodeConfiguration& config)
        : UWRTActionNode(name, config) { }

    static BT::PortsList providedPorts() {
        return {
            UwrtInput("object_name", "Detector class to center"),
            UwrtInput("min_score", "Ignore weaker detections; default 0.2"),
            UwrtInput("max_age_secs", "Lost once no detection is this old; default 1.5"),
            UwrtInput("timeout_secs", "Give up after; default 8"),
            UwrtInput("yaw_tolerance", "Centered within this (rad); default 0.05"),
            UwrtInput("depth_tolerance", "Level within this (m); default 0.1"),
            UwrtInput("center_depth", "false: yaw only; default true"),
            UwrtInput("min_depth", "Shallowest depth to move to; default -0.75"),
            UwrtInput("max_depth", "Deepest depth to move to; default -2.5"),
            UwrtInput("base_frame", "Vehicle frame; default talos/base_link"),
            UwrtInput("align_yaw", "Move square on to the object (its frame's yaw); false: turn in place; default true"),
            UwrtInput("object_frame", "Frame whose yaw to align with; default <object_name>_frame"),
            UwrtInput("standoff", "Camera distance in front of it (m); default the range it was seen from"),
            UwrtInput("position_tolerance", "There within this (m); default 0.1"),
            UwrtInput("align_tolerance", "Square on within this (rad); default 0.08")
        };
    }

    void rosInit() override {
        linearPub = rosnode->create_publisher<riptide_msgs2::msg::ControllerCommand>(CONTROL_LINEAR_TOPIC, 10);
        angularPub = rosnode->create_publisher<riptide_msgs2::msg::ControllerCommand>(CONTROL_ANGULAR_TOPIC, 10);
        odomSub = rosnode->create_subscription<nav_msgs::msg::Odometry>(ODOMETRY_TOPIC, 10,
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex);
                odom = *msg;
            });
        detectionSub = rosnode->create_subscription<vision_msgs::msg::Detection3DArray>(DETECTIONS_TOPIC, 10,
            [this](vision_msgs::msg::Detection3DArray::SharedPtr msg) { onDetections(*msg); });
    }

    BT::NodeStatus onStart() override {
        std::lock_guard<std::mutex> lock(mutex);
        object = tryGetRequiredInput<std::string>(this, "object_name", "");
        minScore = tryGetOptionalInput<double>(this, "min_score", 0.2);
        maxAge = tryGetOptionalInput<double>(this, "max_age_secs", 1.5);
        timeout = tryGetOptionalInput<double>(this, "timeout_secs", 8);
        yawTolerance = tryGetOptionalInput<double>(this, "yaw_tolerance", 0.05);
        depthTolerance = tryGetOptionalInput<double>(this, "depth_tolerance", 0.1);
        centerDepth = tryGetOptionalInput<std::string>(this, "center_depth", "true") != "false";
        minDepth = tryGetOptionalInput<double>(this, "min_depth", -0.75);
        maxDepth = tryGetOptionalInput<double>(this, "max_depth", -2.5);
        baseFrame = tryGetOptionalInput<std::string>(this, "base_frame", "talos/base_link");
        alignYaw = tryGetOptionalInput<std::string>(this, "align_yaw", "true") != "false";
        objectFrame = tryGetOptionalInput<std::string>(this, "object_frame", object + "_frame");
        const double standoffPort = tryGetOptionalInput<double>(this, "standoff", -1);
        standoff = standoffPort > 0 ? std::optional<double>(standoffPort) : std::nullopt;
        positionTolerance = tryGetOptionalInput<double>(this, "position_tolerance", 0.1);
        alignTolerance = tryGetOptionalInput<double>(this, "align_tolerance", 0.08);
        if(maxDepth > minDepth) {
            std::swap(maxDepth, minDepth);
        }
        if(object.empty() || !(maxAge > 0) || !(timeout > 0) || !(yawTolerance > 0) || !(depthTolerance > 0) ||
           !(positionTolerance > 0) || !(alignTolerance > 0)) {
            RCLCPP_ERROR(rosnode->get_logger(), "CenterDetection needs object_name and positive ages, timeout and tolerances");
            return BT::NodeStatus::FAILURE;
        }
        started = rosnode->get_clock()->now();
        hold.reset();
        target.reset(); // only detections from now on
        active = true;
        RCLCPP_INFO(rosnode->get_logger(), "Chasing %s: centering it%s", object.c_str(), alignYaw ? ", square on" : "");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        std::lock_guard<std::mutex> lock(mutex);
        const rclcpp::Time now = rosnode->get_clock()->now();
        if((now - started).seconds() > timeout) {
            RCLCPP_WARN(rosnode->get_logger(), "CenterDetection: %s not centered in %.0f s", object.c_str(), timeout);
            return finish(BT::NodeStatus::FAILURE);
        }
        if(!target || (now - target->stamp).seconds() > maxAge || !odom) {
            if((now - started).seconds() < maxAge) {
                return BT::NodeStatus::RUNNING; // the detection that brought us here may not have been converted yet
            }
            RCLCPP_WARN(rosnode->get_logger(), "CenterDetection: lost %s", object.c_str());
            return finish(BT::NodeStatus::FAILURE);
        }

        const auto& p = odom->pose.pose.position;
        const double yaw = toRPY(odom->pose.pose.orientation).z;
        if(!hold) {
            hold = p;
        }
        // Yaw that puts the target on the camera axis: its bearing, less the axis' yaw in the vehicle and the camera's
        // sideways offset from the vehicle center at that range
        const double dx = target->x - p.x, dy = target->y - p.y, range = std::hypot(dx, dy);
        const double side = range > 1e-3 ? std::clamp(target->cameraSide / range, -1.0, 1.0) : 0.0;
        const double wanted = std::atan2(dy, dx) - target->cameraYaw - std::asin(side);
        const double depth = std::clamp(target->z - target->cameraHeight, maxDepth, minDepth);

        // Where to be: the camera on the object's front normal, standoff out, so facing it square on (+x along its yaw
        // turned round) centers it; until its frame is known, stay put
        geometry_msgs::msg::Point goal = *hold;
        std::optional<double> squareYaw;
        geometry_msgs::msg::TransformStamped objectPose;
        if(alignYaw && lookupTransform(objectFrame, odom->header.frame_id, objectPose)) {
            const double objectYaw = toRPY(objectPose.transform.rotation).z;
            squareYaw = objectYaw + M_PI - target->cameraYaw;
            if(!standoff) {
                const double camX = p.x + std::cos(yaw) * target->cameraX - std::sin(yaw) * target->cameraY;
                const double camY = p.y + std::sin(yaw) * target->cameraX + std::cos(yaw) * target->cameraY;
                standoff = std::hypot(target->x - camX, target->y - camY);
            }
            const double camX = target->x + *standoff * std::cos(objectYaw), camY = target->y + *standoff * std::sin(objectYaw);
            goal.x = camX - (std::cos(*squareYaw) * target->cameraX - std::sin(*squareYaw) * target->cameraY);
            goal.y = camY - (std::sin(*squareYaw) * target->cameraX + std::cos(*squareYaw) * target->cameraY);
        }

        riptide_msgs2::msg::ControllerCommand linear, angular;
        linear.mode = angular.mode = riptide_msgs2::msg::ControllerCommand::POSITION;
        linear.setpoint_vect.x = goal.x;
        linear.setpoint_vect.y = goal.y;
        linear.setpoint_vect.z = centerDepth ? depth : hold->z;
        linearPub->publish(linear);
        angular.setpoint_vect.z = std::remainder(wanted, 2 * M_PI);
        angular.setpoint_quat = toQuat(angular.setpoint_vect); // level
        angularPub->publish(angular);

        const bool yawDone = std::abs(std::remainder(wanted - yaw, 2 * M_PI)) < yawTolerance &&
            std::abs(odom->twist.twist.angular.z) < 0.1;
        const bool depthDone = !centerDepth || std::abs(p.z - depth) < depthTolerance;
        const bool squareDone = !alignYaw || (squareYaw && std::hypot(p.x - goal.x, p.y - goal.y) < positionTolerance &&
            std::abs(std::remainder(*squareYaw - yaw, 2 * M_PI)) < alignTolerance);
        if(yawDone && depthDone && squareDone) {
            RCLCPP_INFO(rosnode->get_logger(), "%s centered%s", object.c_str(), alignYaw ? ", square on" : "");
            active = false;
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
        std::lock_guard<std::mutex> lock(mutex);
        finish(BT::NodeStatus::IDLE);
    }

private:
    struct Target {
        double x, y, z;      // in the odometry frame (smoothed over detections)
        double cameraYaw;    // camera axis yaw in the vehicle frame
        double cameraSide;   // camera position left of the vehicle center, across its axis
        double cameraHeight; // camera above the vehicle center
        double cameraX, cameraY; // camera position in the vehicle frame
        rclcpp::Time stamp;  // when it was last detected (receive time)
    };

    void onDetections(const vision_msgs::msg::Detection3DArray& msg) {
        std::lock_guard<std::mutex> lock(mutex);
        if(!active || !odom) {
            return;
        }
        const vision_msgs::msg::ObjectHypothesisWithPose* best = nullptr;
        for(const auto& detection : msg.detections) {
            for(const auto& result : detection.results) {
                if(result.hypothesis.class_id == object && result.hypothesis.score >= minScore &&
                   (!best || result.hypothesis.score > best->hypothesis.score)) {
                    best = &result;
                }
            }
        }
        if(!best) {
            return;
        }
        const std::string& camera = msg.header.frame_id, &world = odom->header.frame_id;
        geometry_msgs::msg::TransformStamped toWorld, cameraInBase;
        try {
            // At the detection's stamp: the vehicle may be turning
            toWorld = tfBuffer->lookupTransform(world, camera, tf2_ros::fromMsg(msg.header.stamp), tf2::durationFromSec(0.05));
        } catch(tf2::TransformException&) {
            if(!lookupTransform(camera, world, toWorld)) {
                return;
            }
        }
        if(!lookupTransform(camera, baseFrame, cameraInBase)) {
            return;
        }
        geometry_msgs::msg::PointStamped inCamera, inWorld;
        inCamera.point = best->pose.pose.position;
        tf2::doTransform(inCamera, inWorld, toWorld);

        // Camera axis: +z for an optical frame, else +x; its yaw in the vehicle and the camera's sideways offset
        geometry_msgs::msg::Vector3Stamped axis, axisInBase;
        (camera.find("optical") != std::string::npos ? axis.vector.z : axis.vector.x) = 1.0;
        tf2::doTransform(axis, axisInBase, cameraInBase);
        const double ax = axisInBase.vector.x, ay = axisInBase.vector.y;
        if(std::hypot(ax, ay) < 0.5) {
            RCLCPP_WARN_THROTTLE(rosnode->get_logger(), *rosnode->get_clock(), 2000,
                "CenterDetection: %s does not look forward, can't center by yawing", camera.c_str());
            return;
        }
        const double cameraYaw = std::atan2(ay, ax);
        const auto& c = cameraInBase.transform.translation;
        const double side = -std::sin(cameraYaw) * c.x + std::cos(cameraYaw) * c.y;

        const rclcpp::Time now = rosnode->get_clock()->now();
        if(target && (now - target->stamp).seconds() < maxAge) {
            const double a = 0.3; // smooth detection noise
            target->x += a * (inWorld.point.x - target->x);
            target->y += a * (inWorld.point.y - target->y);
            target->z += a * (inWorld.point.z - target->z);
        } else {
            target = Target{inWorld.point.x, inWorld.point.y, inWorld.point.z, 0, 0, 0, 0, 0, now};
        }
        target->cameraYaw = cameraYaw;
        target->cameraSide = side;
        target->cameraHeight = c.z;
        target->cameraX = c.x;
        target->cameraY = c.y;
        target->stamp = now;
    }

    BT::NodeStatus finish(BT::NodeStatus status) {
        // Stop where we are, still facing what we were turning to
        if(active && odom) {
            riptide_msgs2::msg::ControllerCommand linear, angular;
            linear.mode = angular.mode = riptide_msgs2::msg::ControllerCommand::POSITION;
            linear.setpoint_vect = pointToVector3(odom->pose.pose.position);
            linearPub->publish(linear);
            angular.setpoint_vect.z = toRPY(odom->pose.pose.orientation).z;
            angular.setpoint_quat = toQuat(angular.setpoint_vect);
            angularPub->publish(angular);
        }
        active = false;
        target.reset();
        return status;
    }

    std::mutex mutex;
    bool active = false, centerDepth = true, alignYaw = true;
    std::string object, baseFrame, objectFrame;
    std::optional<double> standoff;
    double positionTolerance = 0.1, alignTolerance = 0.08;
    double minScore = 0.2, maxAge = 1.5, timeout = 8, yawTolerance = 0.05, depthTolerance = 0.1, minDepth = -0.75, maxDepth = -2.5;
    rclcpp::Time started;
    std::optional<nav_msgs::msg::Odometry> odom;
    std::optional<geometry_msgs::msg::Point> hold;
    std::optional<Target> target;
    rclcpp::Publisher<riptide_msgs2::msg::ControllerCommand>::SharedPtr linearPub, angularPub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detectionSub;
};
