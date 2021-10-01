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
    latestLocMessage = msg;
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
        InputPort<double>("x"),
        InputPort<double>("y"),
        InputPort<double>("z"),
        InputPort<std::string>("orientation")
    };
}

/**
 * Executes the command
 */
NodeStatus big_move_state::tick() {
    while(ros::ok()) {
        //run onEnter if this is the first time this command is being ticked
        if(!hasEntered) {
            onEnter();
            hasEntered = true;
        }

        positionPublisher.publish(goal.position);
        orientationPublisher.publish(goal.orientation);

        //the controller is currently handling movement (was stated in onEnter()).
        //here, we basically check to see if the position is acheived.
        //when the robot is steady within the target threshold, command ends
        if(locExists) { //if latest loc message exists...
            nav_msgs::Odometry::ConstPtr pos = latestLocMessage; //current localization data
            geometry_msgs::Point position = pos->pose.pose.position;

            bool
                xGood = abs(position.x - goal.position.x) < threshold,
                yGood = abs(position.y - goal.position.y) < threshold,
                zGood = abs(position.z - goal.position.z) < threshold,
                isSteady = steady;

            if(xGood && yGood && zGood && isSteady) {
                break; 
            }
        } else {
            // ROS_ERROR("Localization Message %s not there!", locTopic.c_str());
        }

        ros::spinOnce();
    }

    ROS_INFO("command finished!");
    return NodeStatus::SUCCESS; // command finishes!
}


/**
 * Called just before the first tick is run
 * Returns true if successful, false otherwise.
 */
void big_move_state::onEnter() {
    ros::AsyncSpinner spinner(1);
    spinner.start();
  
    ROS_INFO("Robot is %s", (steady ? std::string("Steady").c_str() : std::string("NOT STEADY").c_str()));

    //assign goal odometry data
    x = getInput<double>("x").value();
    y = getInput<double>("y").value();
    z = getInput<double>("z").value();
    orientation = Util::quaternionFromString(getInput<std::string>("orientation").value());

    ROS_INFO("Moving to position %f, %f, %f with orientation %s", x, y, z, getInput<std::string>("orientation").value().c_str());

    //assign data to goal pose
    goal.orientation.w = 1;
    goal.position.x = x;
    goal.position.y = y;
    goal.position.z = z;
}
