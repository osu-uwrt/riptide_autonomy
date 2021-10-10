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
    return NodeStatus::SUCCESS;
}