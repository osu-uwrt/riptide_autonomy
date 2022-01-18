#include "states.h"

/**
 * State that drives the robot with specified XYZ velocities.
 */

using namespace BT;
using namespace states;

void velocity_state::init() {
    //runtime vars
    hasEntered = false;
    locDataExists = false;

    //publishers
    velocityPublisher = n.advertise<geometry_msgs::Vector3>(velocityTopic, CACHE_SIZE);
    orientationPublisher = n.advertise<geometry_msgs::Quaternion>(orientationTopic, CACHE_SIZE);
    
    //subscriber
    locSub = n.subscribe(locTopic, CACHE_SIZE, &velocity_state::locCallback, this);
}


void velocity_state::locCallback(const nav_msgs::Odometry::ConstPtr& data) {
    currentPose = data->pose.pose;
    locDataExists = true;
}


PortsList velocity_state::providedPorts() {
    return {
        InputPort<std::string>("time"),
        InputPort<std::string>("x_velocity"),
        InputPort<std::string>("y_velocity"),
        InputPort<std::string>("z_velocity")
    };
}


NodeStatus velocity_state::tick() {
    while(ros::ok()) {
        if(!hasEntered) {
            //initial calculations for movement
            startTime = ros::Time::now().sec;
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
            ROS_INFO("Waiting for odometry/filtered...");
            while(!locDataExists) {
                ros::Duration(0.25).sleep();
            }

            //grab orientation to use while moving
            orientation.w = currentPose.orientation.w;
            orientation.x = currentPose.orientation.x;
            orientation.y = currentPose.orientation.y;
            orientation.z = currentPose.orientation.z;
        }
        
        //publish velocity data
        velocityPublisher.publish(velocities);
        orientationPublisher.publish(orientation);

        //get time data from port
        Optional<std::string> timeStr = getInput<std::string>("time");
        int goalTime = std::stoi(timeStr.value());
        int elapsedTime = ros::Time::now().sec - startTime;

        if(elapsedTime >= goalTime) { //the command be done
            ros::Duration(1).sleep();
            geometry_msgs::Vector3 zeroVelocity;
            velocityPublisher.publish(zeroVelocity);
            return NodeStatus::SUCCESS;
        }
    }

    return NodeStatus::FAILURE;
}