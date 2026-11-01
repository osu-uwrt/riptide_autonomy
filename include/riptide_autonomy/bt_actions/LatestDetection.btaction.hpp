#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"
#include <mutex>
#include <optional>
#include <unordered_map>

// Where the detector last saw object_name, in `frame` (default world): the detection's position looked up through TF at
// its own stamp, so a turning vehicle doesn't smear it. SUCCESS with x/y/z if that detection is at most max_age_secs old,
// FAILURE otherwise. Lets a tree aim a camera at what it actually sees while mapping still holds those detections back
// (a new target needs a few agreeing detections before its mapping frame moves), instead of at a stale estimate.
class LatestDetection : public UWRTActionNode {
public:
    LatestDetection(const std::string& name, const BT::NodeConfiguration& config)
        : UWRTActionNode(name, config) { }

    static BT::PortsList providedPorts() {
        return {
            UwrtInput("object_name", "Detector class"),
            UwrtInput("frame", "Frame for the output position; default world"),
            UwrtInput("max_age_secs", "Fail if the latest detection is older; default 1.0"),
            UwrtInput("min_score", "Ignore weaker detections; default 0.2"),
            UwrtOutput("out_x"),
            UwrtOutput("out_y"),
            UwrtOutput("out_z")
        };
    }

    void rosInit() override {
        detectionSource = RosEntityCache::subscription<vision_msgs::msg::Detection3DArray>(rosnode, DETECTIONS_TOPIC, 10);
        detectionListener = detectionSource->listen(
            [this](vision_msgs::msg::Detection3DArray::SharedPtr msg) { onDetections(*msg); });
    }

    BT::NodeStatus onStart() override {
        const std::string object = tryGetRequiredInput<std::string>(this, "object_name", "");
        const std::string frame = tryGetOptionalInput<std::string>(this, "frame", "world");
        const double maxAge = tryGetOptionalInput<double>(this, "max_age_secs", 1.0);
        const double minScore = tryGetOptionalInput<double>(this, "min_score", 0.2);

        Seen seen;
        {
            std::lock_guard<std::mutex> lock(mutex);
            const auto found = latest.find(object);
            if(found == latest.end()) {
                return BT::NodeStatus::FAILURE;
            }
            seen = found->second;
        }
        if(seen.score < minScore || (rosnode->get_clock()->now() - seen.received).seconds() > maxAge) {
            return BT::NodeStatus::FAILURE;
        }

        geometry_msgs::msg::TransformStamped toFrame;
        try {
            toFrame = tfBuffer->lookupTransform(frame, seen.frameId, tf2_ros::fromMsg(seen.stamp), tf2::durationFromSec(0.05));
        } catch(tf2::TransformException&) {
            RCLCPP_WARN_THROTTLE(rosnode->get_logger(), *rosnode->get_clock(), 2000,
                "LatestDetection: no transform at the detection's stamp, using the latest");
            if(!lookupTransform(seen.frameId, frame, toFrame)) {
                return BT::NodeStatus::FAILURE;
            }
        }
        geometry_msgs::msg::PointStamped inCamera, inFrame;
        inCamera.point = seen.position;
        tf2::doTransform(inCamera, inFrame, toFrame);
        postOutput<double>(this, "out_x", inFrame.point.x);
        postOutput<double>(this, "out_y", inFrame.point.y);
        postOutput<double>(this, "out_z", inFrame.point.z);
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus onRunning() override {
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override { }

private:
    struct Seen {
        geometry_msgs::msg::Point position; // in the camera frame
        std::string frameId;
        builtin_interfaces::msg::Time stamp;
        rclcpp::Time received;
        double score;
    };

    void onDetections(const vision_msgs::msg::Detection3DArray& msg) {
        // the strongest detection of each class in this frame (min_score is applied when the node ticks)
        std::unordered_map<std::string, const vision_msgs::msg::ObjectHypothesisWithPose*> best;
        for(const auto& detection : msg.detections) {
            for(const auto& result : detection.results) {
                const auto& id = result.hypothesis.class_id;
                if(!best.count(id) || result.hypothesis.score > best[id]->hypothesis.score) {
                    best[id] = &result;
                }
            }
        }
        const rclcpp::Time now = rosnode->get_clock()->now();
        std::lock_guard<std::mutex> lock(mutex);
        for(const auto& [id, result] : best) {
            latest[id] = Seen{result->pose.pose.position, msg.header.frame_id, msg.header.stamp, now, result->hypothesis.score};
        }
    }

    std::mutex mutex;
    std::unordered_map<std::string, Seen> latest;
    std::shared_ptr<SharedSubscription<vision_msgs::msg::Detection3DArray>> detectionSource;
    RosListenerPtr detectionListener;
};
