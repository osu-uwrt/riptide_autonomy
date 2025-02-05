#pragma once

#include "riptide_autonomy/autonomy_base.hpp"
#include "riptide_autonomy/uwrt_node_types.hpp"

#include <robot_localization/srv/set_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

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
            UwrtInput("x", PORT_OPTIONAL,
                "Optional input; if populated, EKF will be set to this coordinate in the X axis."),

            UwrtInput("y", PORT_OPTIONAL,
                "Optional input; if populated, EKF will be set to this coordinate in the Y axis."),

            UwrtInput("z", PORT_OPTIONAL,
                "Optional input; if populated, EKF will be set to this coordinate in the Z axis."),

            UwrtInput("or", PORT_OPTIONAL,
                "Optional input; if populated, EKF will be set to this coordinate in the roll axis. "
                "If another orientation specified, defaults to 0"),

            UwrtInput("op", PORT_OPTIONAL,
                "Optional input; if populated, EKF will be set to this coordinate in the pitch axis. "
                "If another orientation specified, defaults to 0"),

            UwrtInput("oy", PORT_OPTIONAL,
                "Optional input; if populated, EKF will be set to this coordinate in the yaw axis. "
                "If another orientation specified, defaults to 0"),
        };
    }

    /**
     * @brief Initializes ROS peripherals such as publishers, subscribers, actions, services, etc.
     * Anything requiring the ROS node handle to construct should be initialized here. Do not do it in the 
     * constructor or you will be very sad
     */
    void rosInit() override {
        poseClient = rosNode()->create_client<SetPose>(SET_POSE_SERVER_NAME);
        odomSub = rosNode()->create_subscription<Odometry>(ODOMETRY_TOPIC, 10,std::bind(&PublishEKFPose::recieveOdom, this, _1));
               
    }

    /**
     * @brief Called when the node runs for the first time. If it returns RUNNING, node becomes async
     * @return NodeStatus status of the node after execution
     */
    BT::NodeStatus onStart() override {
        //which metric are we publishing?


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

        //wait until an odometry message is recieved
        if(!odom_recieved){
            return BT::NodeStatus::RUNNING;
        }

        //request
        auto request = std::make_shared<SetPose::Request>();

        //fillout header
        request->pose.header.stamp = rosNode()->get_clock()->now();
        request->pose.header.frame_id = "odom";

        //fill out orientation
        if(tryGetRequiredInput<bool>("setOrientation", false)){
            //convert rpy to quat
            geometry_msgs::msg::Vector3 rpy_msg;
            rpy_msg.x = tryGetOptionalInput<double>("roll", 0);
            rpy_msg.y = tryGetOptionalInput<double>("pitch", 0);
            rpy_msg.z = tryGetOptionalInput<double>("yaw", 0);
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
        if(tryGetRequiredInput<bool>("setX", false)){
            request->pose.pose.pose.position.x = tryGetOptionalInput<double>("x", 0);
        }else{
            request->pose.pose.pose.position.x = odom_msg.pose.pose.position.x;
        }

        //fill out y
        if(tryGetRequiredInput<bool>("setY", false)){
            request->pose.pose.pose.position.y = tryGetOptionalInput<double>("y", 0);
        }else{
            request->pose.pose.pose.position.y = odom_msg.pose.pose.position.y;
        }

        //fill out z
        if(tryGetRequiredInput<bool>("setZ", false)){
            request->pose.pose.pose.position.z = tryGetOptionalInput<double>("z", 0);
        }else{            
            request->pose.pose.pose.position.z = odom_msg.pose.pose.position.z;
        }


        request->pose.pose.covariance = {0.0};

        //send out, and recieve result
        auto result = poseClient->async_send_request(request);

        RCLCPP_INFO(rosNode()->get_logger(), "Updating Pose");

        return BT::NodeStatus::SUCCESS;
    }

    /**
     * @brief Called when the node is halted.
     */
    void onHalted() override {
        
    }

    //recieve odometry message
    void recieveOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_msg = *msg;

        odom_recieved = true;
    }


    private:
        rclcpp::Client<SetPose>::SharedPtr poseClient;
        rclcpp::Subscription<Odometry>::SharedPtr odomSub;

        Odometry odom_msg;
        bool odom_recieved = false;

};
