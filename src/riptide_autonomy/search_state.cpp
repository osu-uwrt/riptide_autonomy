#include "states.h"

/**
 * Source file for the search state class.
 */

using namespace BT;
using namespace states;

/**
 * Called by the constructor of the class when the object is constructed.
 * Use this method to create any other publishers or subscribers that you need.
 */
void search_state::init() {
    positionPublisher = n.advertise<geometry_msgs::Vector3>(positionTopic, CACHE_SIZE);
    //Subscribe to whatever publishes the guess data
}

/**
 * Returns the list of ports that the node relies on.
 */
PortsList search_state::providedPorts() {
    return {
        InputPort<std::string>("frame"),
        InputPort<std::string>("target_error")
        InputPort<std::string>("update_ms")
    };
}

/**
 * Runs the state. Note that because this is a synchronous action node
 * (unfortunately asynchronous nodes don't work too well at the moment),
 * this method can only return SUCCESS or FAILURE. You're not allowed
 * to return RUNNING.
 */
NodeStatus search_state::tick() {
    ROS_INFO("BIG Search");
    bool sunnyDay = true;
    std:string frame = getInput<std:string>("frame").value();
    double target_error = stod(getInput<std:String>("target_error").value());
    double update_sec = stod(InputPort<std::string>("update_sec").value());
    //Set the guess point as the point to move toward
    
    //Get a snapshot of the time at the start
    ros::Time begin = ros::Time::now();

    while(sunnyDay){
        if(ros:Time::now()-begin >=ros::Duration(update_sec){

            //Get the new point estimate error and location
            //If the esitmate error < = target_error, return success
            //Else set new target point to the new location

            //If the distance from the robot to the new location is < 1ft, sunnyDay = false;
            begin = ros::Time::now();
        }
    }


    return NodeStatus::SUCCESS;
}