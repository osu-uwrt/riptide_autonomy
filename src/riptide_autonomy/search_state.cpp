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
}

/**
 * Returns the list of ports that the node relies on.
 */
PortsList search_state::providedPorts() {
    return {
        InputPort<std::string>("frame"),
        InputPort<std::string>("target_error")
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
    return NodeStatus::SUCCESS;
}