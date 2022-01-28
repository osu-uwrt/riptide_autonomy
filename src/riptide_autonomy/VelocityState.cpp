#include "autonomy.h"

using namespace BT;
using namespace std::placeholders;


void VelocityState::init(rclcpp::Node::SharedPtr node) {
    rosnode = node;

    velocityPub = node->create_publisher<geometry_msgs::msg::Vector3>(LINEAR_VELOCITY_TOPIC, 10);
    orientationPub = node->create_publisher<geometry_msgs::msg::Quaternion>(ORIENTATION_TOPIC, 10);
    odomSub = node->create_subscription<nav_msgs::msg::Odometry>(ODOMETRY_TOPIC, 10, std::bind(&VelocityState::odomCallback, this, _1));
}


NodeStatus VelocityState::tick() {
    while(rclcpp::ok()) {
        if(!hasEntered) {
            //initial calculations for movement
            startTime = rosnode->now().seconds();
            hasEntered = true;

            //figure out velocities to drive the robot at
            Optional<std::string> 
                xVelStr = getInput<std::string>("x_velocity"),
                yVelStr = getInput<std::string>("y_velocity"),
                zVelStr = getInput<std::string>("z_velocity");
            
            //grab velocities from behaviotree parameters
            double 
                xVel = std::stod(xVelStr.value()),
                yVel = std::stod(yVelStr.value()),
                zVel = std::stod(zVelStr.value());
            
            //assemble Vector3 that will be published while robot is moving
            velocities.x = xVel;
            velocities.y = yVel;
            velocities.z = zVel;

            //wait for localization data so that we can grab the orientation to lock to
            RCLCPP_INFO(log, "Waiting for odometry/filtered...");
            while(!odomDataExists) {
                rclcpp::spin_some(rosnode);
            }

            //grab orientation to use while moving
            orientation.w = currentPose.orientation.w;
            orientation.x = currentPose.orientation.x;
            orientation.y = currentPose.orientation.y;
            orientation.z = currentPose.orientation.z;
        }
        
        //publish velocity data
        velocityPub->publish(velocities);
        orientationPub->publish(orientation);

        //get time data from port
        Optional<std::string> timeStr = getInput<std::string>("time");
        int goalTime = std::stoi(timeStr.value());
        int elapsedTime = rosnode->now().seconds() - startTime;

        if(elapsedTime >= goalTime) { //the command be done
            geometry_msgs::msg::Vector3 zeroVelocity;
            velocityPub->publish(zeroVelocity);
            return NodeStatus::SUCCESS;
        }
    }

    return NodeStatus::FAILURE;
}


void VelocityState::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    this->currentPose = msg->pose.pose;
    odomDataExists = true;
}
