#pragma once

#include "riptide_autonomy/autonomy_lib.hpp"
#include <robot_localization/srv/set_pose.h>
#include <algorithm>
#include <cmath>
#include <mutex>

class PublishEKFPose : public UWRTActionNode {
    using SetPose = robot_localization::srv::SetPose;
    using Odometry = nav_msgs::msg::Odometry;

    public:
    PublishEKFPose(const std::string& name, const BT::NodeConfiguration& config)
    : UWRTActionNode(name, config) {
        
    }

    /**
     * @brief Declares ports needed by this node.
     * @return PortsList Needed ports.
     */
    static BT::PortsList providedPorts() {
        return {
            UwrtInput("x"),
            UwrtInput("setX"),
            UwrtInput("y"),
            UwrtInput("setY"),
            UwrtInput("z"),
            UwrtInput("setZ"),
            UwrtInput("roll"),
            UwrtInput("pitch"),
            UwrtInput("yaw"),
            UwrtInput("setOrientation"),
            UwrtInput("timeout_secs", "Reset and odometry confirmation timeout (default 5 seconds)"),
            UwrtInput("position_tolerance", "Reset confirmation tolerance in meters (default 0.1)"),
            UwrtInput("orientation_tolerance", "Reset confirmation tolerance in radians (default 0.1)")
        };
    }

    /**
     * @brief Initializes ROS peripherals such as publishers, subscribers, actions, services, etc.
     * Anything requiring the ROS node handle to construct should be initialized here. Do not do it in the 
     * constructor or you will be very sad
     */
    void rosInit() override {
        poseClient = rosnode->create_client<SetPose>(SET_POSE_SERVER_NAME);
        odomSub = rosnode->create_subscription<Odometry>(ODOMETRY_TOPIC, 10,std::bind(&PublishEKFPose::recieveOdom, this, _1));
               
    }

    /**
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override {
        clearPendingRequest();
        requestSent = false;
        responseReceived = false;
        timeoutSecs = tryGetOptionalInput<double>(this, "timeout_secs", 5.0);
        positionTolerance = tryGetOptionalInput<double>(this, "position_tolerance", 0.1);
        orientationTolerance = tryGetOptionalInput<double>(this, "orientation_tolerance", 0.1);
        if (!std::isfinite(timeoutSecs) || timeoutSecs <= 0 ||
            !std::isfinite(positionTolerance) || positionTolerance < 0 ||
            !std::isfinite(orientationTolerance) || orientationTolerance < 0) {
            RCLCPP_ERROR(rosNode()->get_logger(), "Invalid pose reset timeout or tolerance.");
            return BT::NodeStatus::FAILURE;
        }
        {
            std::lock_guard<std::mutex> lock(odomMutex);
            odom_recieved = false;
        }
        startTime = std::chrono::steady_clock::now();

        //wait for client to become available
        if(!poseClient->wait_for_service(1s)) {
            RCLCPP_ERROR(rosNode()->get_logger(), "Set Pose Service: %s is not available.", SET_POSE_SERVER_NAME.c_str());
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;

    }

    /**
     * @brief Called periodically while the node status is RUNNING
     * @return NodeStatus The node status after 
     */
    BT::NodeStatus onRunning() override {
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - startTime).count() > timeoutSecs) {
            RCLCPP_ERROR(rosNode()->get_logger(), "Timed out waiting for pose reset and corrected odometry.");
            clearPendingRequest();
            return BT::NodeStatus::FAILURE;
        }

        // Tree ticks and ROS callbacks run on different threads.
        std::lock_guard<std::mutex> lock(odomMutex);
        if (requestSent) {
            if (!responseReceived) {
                if (result->wait_for(0s) != std::future_status::ready) {
                    return BT::NodeStatus::RUNNING;
                }
                result->get();
                result.reset();
                responseReceived = true;
                // A service reply alone does not mean subscribers have seen the reset.
                odom_recieved = false;
                return BT::NodeStatus::RUNNING;
            }
            if (odom_recieved && correctedOdometry()) {
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::RUNNING;
        }

        //wait until an odometry message is recieved
        if(!odom_recieved){
            return BT::NodeStatus::RUNNING;
        }

        //request
        auto request = std::make_shared<SetPose::Request>();

        //fillout header
        request->pose.header.stamp = rosnode->get_clock()->now();
        request->pose.header.frame_id = "odom";

        //fill out orientation
        setOrientation = tryGetRequiredInput<bool>(this, "setOrientation", false);
        setX = tryGetRequiredInput<bool>(this, "setX", false);
        setY = tryGetRequiredInput<bool>(this, "setY", false);
        setZ = tryGetRequiredInput<bool>(this, "setZ", false);
        if(setOrientation){
            //convert rpy to quat
            geometry_msgs::msg::Vector3 rpy_msg;
            rpy_msg.x = tryGetOptionalInput<double>(this, "roll", 0);
            rpy_msg.y = tryGetOptionalInput<double>(this, "pitch", 0);
            rpy_msg.z = tryGetOptionalInput<double>(this, "yaw", 0);
            geometry_msgs::msg::Quaternion quat_msg = toQuat(rpy_msg);

            request->pose.pose.pose.orientation.w = quat_msg.w;
            request->pose.pose.pose.orientation.x = quat_msg.x;
            request->pose.pose.pose.orientation.y = quat_msg.y;
            request->pose.pose.pose.orientation.z = quat_msg.z;
        }else{
            request->pose.pose.pose.orientation.w = odom_msg.pose.pose.orientation.w;
            request->pose.pose.pose.orientation.x = odom_msg.pose.pose.orientation.x;
            request->pose.pose.pose.orientation.y = odom_msg.pose.pose.orientation.y;
            request->pose.pose.pose.orientation.z = odom_msg.pose.pose.orientation.z;
        }
        
        //fill out x
        if(setX){
            request->pose.pose.pose.position.x = tryGetOptionalInput<double>(this, "x", 0);
        }else{
            request->pose.pose.pose.position.x = odom_msg.pose.pose.position.x;
        }

        //fill out y
        if(setY){
            request->pose.pose.pose.position.y = tryGetOptionalInput<double>(this, "y", 0);
        }else{
            request->pose.pose.pose.position.y = odom_msg.pose.pose.position.y;
        }

        //fill out z
        if(setZ){
            request->pose.pose.pose.position.z = tryGetOptionalInput<double>(this, "z", 0);
        }else{            
            request->pose.pose.pose.position.z = odom_msg.pose.pose.position.z;
        }


        request->pose.pose.covariance = {0.0};

        requestedPose = request->pose;
        result = std::make_unique<rclcpp::Client<SetPose>::FutureAndRequestId>(
            poseClient->async_send_request(request));
        requestSent = true;

        RCLCPP_INFO(rosNode()->get_logger(), "Updating Pose");

        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Called when the node is halted.
     */
    void onHalted() override {
        clearPendingRequest();
    }

    //recieve odometry message
    void recieveOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(odomMutex);
        odom_msg = *msg;

        odom_recieved = true;
    }


    private:
        void clearPendingRequest() {
            if (result) {
                poseClient->remove_pending_request(*result);
                result.reset();
            }
        }

        bool correctedOdometry() const {
            if (odom_msg.header.frame_id != requestedPose.header.frame_id ||
                rclcpp::Time(odom_msg.header.stamp) <= rclcpp::Time(requestedPose.header.stamp)) {
                return false;
            }
            const auto &actual = odom_msg.pose.pose;
            const auto &target = requestedPose.pose;
            const auto near = [this](double a, double b) {
                return std::isfinite(a) && std::abs(a - b) <= positionTolerance;
            };
            if ((setX && !near(actual.position.x, target.pose.position.x)) ||
                (setY && !near(actual.position.y, target.pose.position.y)) ||
                (setZ && !near(actual.position.z, target.pose.position.z))) {
                return false;
            }
            if (setOrientation) {
                tf2::Quaternion actualQ, targetQ;
                tf2::fromMsg(actual.orientation, actualQ);
                tf2::fromMsg(target.pose.orientation, targetQ);
                const double norm = std::sqrt(actualQ.length2() * targetQ.length2());
                if (!std::isfinite(norm) || norm <= 0) {
                    return false;
                }
                // q and -q encode the same orientation.
                const double dot = std::min(1.0, std::abs(actualQ.dot(targetQ)) / norm);
                if (!(2 * std::acos(dot) <= orientationTolerance)) {
                    return false;
                }
            }
            return true;
        }

        rclcpp::Client<SetPose>::SharedPtr poseClient;
        rclcpp::Subscription<Odometry>::SharedPtr odomSub;
        std::unique_ptr<rclcpp::Client<SetPose>::FutureAndRequestId> result;
        geometry_msgs::msg::PoseWithCovarianceStamped requestedPose;
        std::chrono::steady_clock::time_point startTime;
        double timeoutSecs = 5.0, positionTolerance = 0.1, orientationTolerance = 0.1;
        bool requestSent = false, responseReceived = false;
        bool setX = false, setY = false, setZ = false, setOrientation = false;
        std::mutex odomMutex;

        Odometry odom_msg;
        bool odom_recieved = false;

};
