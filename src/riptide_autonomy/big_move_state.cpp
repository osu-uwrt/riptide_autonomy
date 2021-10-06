#include "states.h"

/**
 * Source file for the big_move_state class.
 * 
 * KNOWN ISSUES:
 * - Had to use a while() loop in the tick() method because SyncActionNode really do be like that.
 * There is a problem with BehaviorTree with ROS Noetic specifically where the CoroActionNode
 * (the one we want) has no references to anything. 
 */

using namespace states;
using namespace BT;

void big_move_state::steadyCallback(const std_msgs::Bool::ConstPtr& msg) {
    steady = msg->data;
}

void big_move_state::locCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    latestLocData = msg->pose.pose;
    locExists = true;
}

/**
 * Basically a constructor
 */
void big_move_state::init() {
    //create cached subscriber
    this->steadySubscriber     = n.subscribe(steadyTopic, CACHE_SIZE, &states::big_move_state::steadyCallback, this);
    this->locSubscriber        = n.subscribe(locTopic, CACHE_SIZE, &states::big_move_state::locCallback, this);
    this->positionPublisher    = n.advertise<geometry_msgs::Vector3>(positionTopic, CACHE_SIZE);
    this->orientationPublisher = n.advertise<geometry_msgs::Quaternion>(orientationTopic, CACHE_SIZE);
    
    hasEntered = false;
    steady = false;
}

/**
 * Defines needed ports
 */
PortsList big_move_state::providedPorts() {
    return { 
        InputPort<std::string>("x"),
        InputPort<std::string>("y"),
        InputPort<std::string>("z"),
        InputPort<std::string>("orientation_x"),
        InputPort<std::string>("orientation_y"),
        InputPort<std::string>("orientation_z"),
        InputPort<std::string>("orientation_w")
    };
}

/**
 * Executes the command
 */
NodeStatus big_move_state::tick() {
    // ros::AsyncSpinner spinner(1);
    // spinner.start();
    
    while(ros::ok()) {
        ros::spinOnce();

        //run onEnter if this is the first time this command is being ticked
        if(!hasEntered) { 
            hasEntered = onEnter();
            continue; //this avoids checking if the command is finished if we havent entered yethasEntered = onEnter();
        }

        //commands has entered, goal has been set.
        positionPublisher.publish(goal.position);
        orientationPublisher.publish(goal.orientation);

        //the controller is currently handling movement (was stated in onEnter()).
        //here, we basically check to see if the position is acheived.
        //when the robot is steady within the target threshold, command ends
        if(locExists) { //if latest loc message exists...
            geometry_msgs::Point position = latestLocData.position;

            bool
                xGood = abs(position.x - goal.position.x) < threshold,
                yGood = abs(position.y - goal.position.y) < threshold,
                zGood = abs(position.z - goal.position.z) < threshold;
           
            if(xGood && yGood && zGood && steady) { //if it's steady then the orientation will be good as well
                ROS_INFO("Move finished!");
                break; 
            }
        }
    }

    return NodeStatus::SUCCESS; // command finishes!
}


/**
 * Called just before the first tick is run
 * Returns true if successful, false otherwise.
 */
bool big_move_state::onEnter() {
    //wait for the robot to be steady
    if(!steady) {
        return false;
    }

    //wait for odometry data to exist
    if(!locExists) {
        return false;
    }

    //now that the robot is steady, capture its current odometry data
    geometry_msgs::Pose currentPose = latestLocData;

    //pull inputs from behaviortree
    Optional<std::string> 
        x = getInput<std::string>("x"),
        y = getInput<std::string>("y"),
        z = getInput<std::string>("z"),
        ox = getInput<std::string>("orientation_x"),
        oy = getInput<std::string>("orientation_y"),
        oz = getInput<std::string>("orientation_z"),
        ow = getInput<std::string>("orientation_w");
    
    double
        xValue = std::stod(x.value()),
        yValue = std::stod(y.value()),
        zValue = std::stod(z.value()),
        oxValue = std::stod(ox.value()),
        oyValue = std::stod(oy.value()),
        ozValue = std::stod(oz.value()),
        owValue = std::stod(ow.value());

    //assign position data to goal pose
    if(x.has_value() && y.has_value() && z.has_value()) { //position fully defined, set it as goal position
        goal.position.x = xValue;
        goal.position.y = yValue;
        goal.position.z = zValue;
    } else { // position not fully defined, fall back to current position
        ROS_INFO("Goal Position not fully specified. Falling back to current position.");
        goal.position = currentPose.position;
    }

    //assign orientation data to goal pose
    if(ox.has_value() && oy.has_value() && oz.has_value() && ow.has_value()) { //orientation fully defined
        goal.orientation.x = oxValue;
        goal.orientation.y = oyValue;
        goal.orientation.z = ozValue;
        goal.orientation.w = owValue;
    } else { //orientation not fully defined, fall back to current orientation
        ROS_INFO("Goal Orientation not fully specified. Falling back to current orientation");
        goal.orientation = currentPose.orientation;
    }

    ROS_INFO (
        "Moving to position %f, %f, %f with orientation %f, %f, %f, %f",
        goal.position.x,
        goal.position.y,
        goal.position.z,
        goal.orientation.x,
        goal.orientation.y,
        goal.orientation.z,
        goal.orientation.w
    );
    return true;
}
