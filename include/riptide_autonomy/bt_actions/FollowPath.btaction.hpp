#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"
#include <riptide_msgs2/action/follow_path.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <algorithm>
#include <sstream>

/**
 * Flies through a list of waypoints with the MPC's follow_path action: it passes
 * each waypoint without stopping and settles only on the last. Waypoints are
 * "x,y,z,yaw; x,y,z,yaw; ..." (meters, radians) in `frame`, or with roll,pitch,yaw
 * as six values. A waypoint written "some_frame: x,y,z,yaw" is in some_frame
 * instead, so one path can span several objects. {blackboard} references are
 * filled in. Depth is clamped to [max_depth, min_depth] in world like
 * PrimitiveMovePosition.
 *
 * Options after a waypoint, each "| key=value" (riptide_msgs2/PathSegment; points
 * in the waypoint's frame), say how the path gets there:
 *   arc=cx,cy,sweep       round (cx, cy) by sweep rad (+ = counterclockwise from above)
 *   heading=path|look_at  face the direction of travel / look_at (default: blend to yaw)
 *   look_at=x,y,z         yaw_offset=rad          spin=rad (extra yaw over the leg)
 *   spin_rate=rad/m       steady spin per metre (the spin_rate port sets it for every leg)
 * AddWaypoint and AddArc write these.
 */
class FollowPath : public UWRTActionNode {
    using Action = riptide_msgs2::action::FollowPath;
    using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;

    public:
    FollowPath(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config) {

    }

    /**
     * @brief Declares ports needed by this node.
     * @return PortsList Needed ports.
     */
    static BT::PortsList providedPorts() {
        return {
            UwrtInput("frame", "Frame of waypoints without their own (default world)"),
            UwrtInput("waypoints", "\"x,y,z,yaw; frame: x,y,z,yaw | arc=cx,cy,sweep; ...\" (or x,y,z,roll,pitch,yaw); {blackboard} references allowed"),
            UwrtInput("min_depth", "Shallowest allowed z in world (default -0.55)"),
            UwrtInput("max_depth", "Deepest allowed z in world (default -4)"),
            UwrtInput("timeout", "Seconds before giving up (default 60)"),
            UwrtInput("spin_rate", "Steady spin for the whole path, rad per metre (about rad/s / cruise speed; default 0)")
        };
    }

    /**
     * @brief Initializes ROS peripherals such as publishers, subscribers, actions, services, etc.
     * Anything requiring the ROS node handle to construct should be initialized here. Do not do it in the
     * constructor or you will be very sad
     */
    void rosInit() override {
        client = rclcpp_action::create_client<Action>(rosnode, "follow_path");
    }

    /**
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override {
        goalFuture = {};
        resultFuture = {};
        goalHandle.reset();
        goal = Action::Goal();
        startTime = rosnode->get_clock()->now();
        timeout = tryGetOptionalInput<double>(this, "timeout", 60);

        const std::string frame = tryGetOptionalInput<std::string>(this, "frame", "world");
        const std::string text = formatStringWithBlackboard(tryGetRequiredInput<std::string>(this, "waypoints", ""), this);
        minDepth = tryGetOptionalInput<double>(this, "min_depth", -0.55);
        maxDepth = tryGetOptionalInput<double>(this, "max_depth", -4);
        const double spinRate = tryGetOptionalInput<double>(this, "spin_rate", 0);
        waypointsInFrames.clear();

        std::stringstream waypoints(text);
        std::string item;
        while(std::getline(waypoints, item, ';')) {
            std::string pointFrame = frame;
            const size_t colon = item.find(':');
            if(colon != std::string::npos) {
                pointFrame = item.substr(0, colon);
                pointFrame.erase(0, pointFrame.find_first_not_of(" \t\n"));
                pointFrame.erase(pointFrame.find_last_not_of(" \t\n") + 1);
                item = item.substr(colon + 1);
            }
            std::vector<std::string> options;
            std::stringstream fields(item);
            std::string field;
            std::getline(fields, item, '|');
            while(std::getline(fields, field, '|')) {
                options.push_back(field);
            }
            std::vector<double> v;
            if(!parseNumbers(item, v, text)) {
                return BT::NodeStatus::FAILURE;
            }
            if(v.empty()) {
                continue;
            }
            if(v.size() != 4 && v.size() != 6) {
                RCLCPP_ERROR(rosnode->get_logger(), "FollowPath: waypoint \"%s\" needs x,y,z,yaw or x,y,z,roll,pitch,yaw", item.c_str());
                return BT::NodeStatus::FAILURE;
            }

            geometry_msgs::msg::Pose pose;
            pose.position.x = v[0];
            pose.position.y = v[1];
            pose.position.z = v[2];
            geometry_msgs::msg::Vector3 rpy;
            rpy.x = v.size() == 6 ? v[3] : 0;
            rpy.y = v.size() == 6 ? v[4] : 0;
            rpy.z = v.back();
            pose.orientation = toQuat(rpy);

            Segment segment;
            segment.msg.spin_rate = spinRate;
            for(const std::string& option : options) {
                const size_t equals = option.find('=');
                std::string key = option.substr(0, equals), value = equals == std::string::npos ? "" : option.substr(equals + 1);
                key.erase(0, key.find_first_not_of(" \t\n"));
                key.erase(key.find_last_not_of(" \t\n") + 1);
                value.erase(0, value.find_first_not_of(" \t\n"));
                value.erase(value.find_last_not_of(" \t\n") + 1);
                std::vector<double> numbers;
                if(key == "heading") {
                    if(value == "waypoint") {
                        segment.msg.heading = riptide_msgs2::msg::PathSegment::HEADING_WAYPOINT;
                    } else if(value == "path") {
                        segment.msg.heading = riptide_msgs2::msg::PathSegment::HEADING_PATH;
                    } else if(value == "look_at") {
                        segment.msg.heading = riptide_msgs2::msg::PathSegment::HEADING_LOOK_AT;
                    } else {
                        RCLCPP_ERROR(rosnode->get_logger(), "FollowPath: unknown heading \"%s\"", value.c_str());
                        return BT::NodeStatus::FAILURE;
                    }
                    continue;
                }
                if(!parseNumbers(value, numbers, text)) {
                    return BT::NodeStatus::FAILURE;
                }
                if(key == "arc" && numbers.size() == 3) {
                    segment.msg.shape = riptide_msgs2::msg::PathSegment::ARC;
                    segment.center.x = numbers[0];
                    segment.center.y = numbers[1];
                    segment.msg.sweep = numbers[2];
                } else if(key == "look_at" && numbers.size() == 3) {
                    segment.lookAt.x = numbers[0];
                    segment.lookAt.y = numbers[1];
                    segment.lookAt.z = numbers[2];
                } else if(key == "yaw_offset" && numbers.size() == 1) {
                    segment.msg.yaw_offset = numbers[0];
                } else if(key == "spin" && numbers.size() == 1) {
                    segment.msg.spin = numbers[0];
                } else if(key == "spin_rate" && numbers.size() == 1) {
                    segment.msg.spin_rate = numbers[0];
                } else {
                    RCLCPP_ERROR(rosnode->get_logger(), "FollowPath: bad option \"%s\" in \"%s\"", option.c_str(), text.c_str());
                    return BT::NodeStatus::FAILURE;
                }
            }
            waypointsInFrames.push_back({pointFrame, pose, segment});
        }

        if(waypointsInFrames.empty()) {
            RCLCPP_ERROR(rosnode->get_logger(), "FollowPath: no waypoints in \"%s\"", text.c_str());
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after
     */
    BT::NodeStatus onRunning() override {
        if((rosnode->get_clock()->now() - startTime).seconds() > timeout) {
            RCLCPP_ERROR(rosnode->get_logger(), "FollowPath timed out after %.0f seconds", timeout);
            cancel();
            return BT::NodeStatus::FAILURE;
        }

        // Like TransformPose, give TF a few seconds to have the frames.
        if(goal.path_points.empty() && !resolveWaypoints()) {
            return (rosnode->get_clock()->now() - startTime).seconds() < 3 ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
        }

        if(!goalFuture.valid()) {
            if(!client->action_server_is_ready()) {
                RCLCPP_WARN_THROTTLE(rosnode->get_logger(), *rosnode->get_clock(), 2000,
                    "FollowPath: waiting for the follow_path server (MPC controller)");
                return BT::NodeStatus::RUNNING;
            }
            goalFuture = client->async_send_goal(goal);
            return BT::NodeStatus::RUNNING;
        }

        if(!goalHandle) {
            if(goalFuture.wait_for(0s) != std::future_status::ready) {
                return BT::NodeStatus::RUNNING;
            }
            goalHandle = goalFuture.get();
            if(!goalHandle) {
                RCLCPP_ERROR(rosnode->get_logger(), "FollowPath goal was rejected");
                return BT::NodeStatus::FAILURE;
            }
            resultFuture = client->async_get_result(goalHandle);
        }

        if(resultFuture.wait_for(0s) != std::future_status::ready) {
            return BT::NodeStatus::RUNNING;
        }
        const auto result = resultFuture.get();
        goalHandle.reset();
        if(result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(rosnode->get_logger(), "FollowPath complete");
            return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_ERROR(rosnode->get_logger(), "FollowPath failed (code %d): %s",
            static_cast<int>(result.result ? result.result->error_code : 0),
            result.result ? result.result->error_msg.c_str() : "no result");
        return BT::NodeStatus::FAILURE;
    }

    /**
     * @brief Called when the node is halted.
     */
    void onHalted() override {
        cancel();
    }

    private:
    // Puts every waypoint into world (depth-clamped) once all their frames resolve.
    bool resolveWaypoints() {
        std::vector<geometry_msgs::msg::PoseStamped> points;
        std::vector<riptide_msgs2::msg::PathSegment> segments;
        for(const auto& [frame, local, segment] : waypointsInFrames) {
            geometry_msgs::msg::Pose pose = local;
            riptide_msgs2::msg::PathSegment msg = segment.msg;
            msg.center = segment.center;
            msg.look_at = segment.lookAt;
            if(frame != "world") {
                geometry_msgs::msg::TransformStamped transform;
                if(!lookupTransform(frame, "world", transform)) {
                    return false; // lookupTransform warns (throttled)
                }
                pose = doTransform(pose, transform);
                const auto point = [&](const geometry_msgs::msg::Point& p) {
                    geometry_msgs::msg::Pose in;
                    in.position = p;
                    in.orientation.w = 1;
                    return doTransform(in, transform).position;
                };
                msg.center = point(segment.center);
                msg.look_at = point(segment.lookAt);
                // sweep is about the frame's +z; turn it the other way if that points down in world
                const auto& q = transform.transform.rotation;
                if(1 - 2 * (q.x * q.x + q.y * q.y) < 0) {
                    msg.sweep = -msg.sweep;
                }
            }
            pose.position.z = std::clamp(pose.position.z, maxDepth, minDepth);
            segments.push_back(msg);

            geometry_msgs::msg::PoseStamped point;
            point.header.frame_id = "world";
            point.header.stamp = rosnode->get_clock()->now();
            point.pose = pose;
            points.push_back(point);
            static const char* headings[] = {"", ", heading path", ", looking at"};
            RCLCPP_INFO(rosnode->get_logger(), "FollowPath waypoint %zu (%s): XYZ %.2f, %.2f, %.2f in world%s%s",
                points.size(), frame.c_str(), pose.position.x, pose.position.y, pose.position.z,
                msg.shape == riptide_msgs2::msg::PathSegment::ARC ? ", arc" : "", headings[std::min<int>(msg.heading, 2)]);
        }
        goal.path_points = points;
        goal.segments = segments;
        return true;
    }

    void cancel() {
        if(!goalHandle && goalFuture.valid() && goalFuture.wait_for(0s) == std::future_status::ready) {
            goalHandle = goalFuture.get();
        }
        if(goalHandle) {
            client->async_cancel_goal(goalHandle);
        } else if(goalFuture.valid()) { // still being accepted; this is the only path client
            client->async_cancel_all_goals();
        }
        goalHandle.reset();
        goalFuture = {};
    }

    bool parseNumbers(const std::string& item, std::vector<double>& v, const std::string& text) {
        std::stringstream numbers(item);
        std::string number;
        while(std::getline(numbers, number, ',')) {
            if(number.find_first_not_of(" \t\n") == std::string::npos) {
                continue;
            }
            try {
                v.push_back(std::stod(number));
            } catch(const std::exception&) {
                RCLCPP_ERROR(rosnode->get_logger(), "FollowPath: \"%s\" is not a number in \"%s\"", number.c_str(), text.c_str());
                return false;
            }
        }
        return true;
    }

    struct Segment {
        riptide_msgs2::msg::PathSegment msg; // points filled in once resolved into world
        geometry_msgs::msg::Point center, lookAt; // in the waypoint's frame
    };
    struct WaypointInFrame {
        std::string frame;
        geometry_msgs::msg::Pose pose;
        Segment segment;
    };

    rclcpp_action::Client<Action>::SharedPtr client;
    Action::Goal goal;
    std::shared_future<GoalHandle::SharedPtr> goalFuture;
    GoalHandle::SharedPtr goalHandle;
    std::shared_future<GoalHandle::WrappedResult> resultFuture;
    rclcpp::Time startTime;
    double timeout = 60, minDepth = -0.55, maxDepth = -4;
    std::vector<WaypointInFrame> waypointsInFrames;
};
