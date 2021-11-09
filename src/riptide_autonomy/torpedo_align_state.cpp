#include "states.h"

/**
 * Source file for the torpedo align state.
 */

using namespace BT;
using namespace states;

/**
 * Called when the object is constructed. Use this to create any needed publishers/subscribers
 */
void torpedo_align_state::init() {
    positionPublisher = n.advertise<geometry_msgs::Vector3>(positionTopic, CACHE_SIZE);
}

/**
 * Returns the ports that the state relies on.
 */
PortsList torpedo_align_state::providedPorts() {
    return {
        // needed ports here
    };
}

/**
 * Runs the state. Can only return SUCCESS or FAILURE, never RUNNING
 */
NodeStatus torpedo_align_state::tick() {
    ROS_INFO("BIG Align");
    //We loop until we close
    // while(pos<threshold){
    //     //take image update from our subscriber
    //     //check if its new
    //     // find the centroid of the dark spot
    //     // make that centroid line up with our torpedo mechanism (use TF)
    //     // update puddles/position

    // }
    //fire
    return NodeStatus::SUCCESS;
}