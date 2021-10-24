#include "states.h"

/**
 * Source file for the search state class.
 */

using namespace BT;
using namespace states;

geometry_msgs::Vector3 guessLocation;
double guessError;

/**
 * Called by the constructor of the class when the object is constructed.
 * Use this method to create any other publishers or subscribers that you need.
 */
void search_state::init() {
    positionPublisher = n.advertise<geometry_msgs::Vector3>(positionTopic, CACHE_SIZE);
}

/**
 * Returns the list of ports that the node relies on.
 */
PortsList search_state::providedPorts() {
    return {
        InputPort<std::string>("frame"),
        InputPort<std::string>("target_error")
        InputPort<std::string>("update_sec")
    };
}


void guessCallback(){

}

double distance(geometry_msgs::Vector3 point1, geometry_msgs::Vector3 point2){
    return sqrt(pow(point2.x - point1.x, 2) +pow(point2.y - point1.y, 2) + pow(point2.y - point1.y, 2) * 1.0);
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
    std:string target = getInput<std:string>("target").value();
    std:string frame = target + "_frame";
    double target_error = stod(getInput<std:String>("target_error").value());
    double update_sec = stod(InputPort<std::string>("update_sec").value());

    //Subscribe to the guess place
    guessSubscriber = n.subscribe("mapping/"+target, CACHE_SIZE, &states::big_move_state::locCallback, this);

    geometry_msgs::Vector3 posToMoveTo = guessLocation;
    
    //Get a snapshot of the time at the start
    ros::Time begin = ros::Time::now();

    while(sunnyDay){
        if(posToMoveTo){
            if(ros:Time::now()-begin >=ros::Duration(update_sec){
                posToMoveTo = guessLocation;
                double newGuessError = guessError;
                if(newGuessError <= target_error){
                    return NodeStatus::SUCCESS;
                } else {
                    //publish guess to move
                }

            //If the distance from the robot to the new location is < 1ft, sunnyDay = false;

            if(distance(guessLocation, thisLocation)<=1){
                sunnyDay = false;
            }
            begin = ros::Time::now();
            }
        }
        
    } else {
        posToMoveTo = guessLocation;
    }


    return NodeStatus::FAILURE;
}