#include "states.h"

/**
 * State that drives the robot with specified XYZ velocities.
 */

using namespace BT;
using namespace states;

void velocity_state::init() {
    hasEntered = false;
    velocityPublisher = n.advertise<geometry_msgs::Vector3>(velocityTopic, CACHE_SIZE);
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
            startTime = ros::Time::now().sec;
            hasEntered = true;

            //figure out velocities to drive the robot at
            Optional<std::string> 
                xVelStr = getInput<std::string>("x_velocity"),
                yVelStr = getInput<std::string>("y_velocity"),
                zVelStr = getInput<std::string>("z_velocity");
            
            double 
                xVel = std::stod(xVelStr.value()),
                yVel = std::stod(yVelStr.value()),
                zVel = std::stod(zVelStr.value());
            
            //assemble Vector3
            velocities.x = xVel;
            velocities.y = yVel;
            velocities.z = zVel;
        }
        
        //publish velocity data
        velocityPublisher.publish(velocities);

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