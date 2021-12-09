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
    this->angVelocityPublisher = n.advertise<geometry_msgs::Vector3>(angVelocityTopic, CACHE_SIZE);
    
    hasEntered = false;
    steady = false;
    publishPosition = true;
    publishOrientation = true;
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
        InputPort<std::string>("orientation_w"),
        InputPort<std::string>("v_roll"),
        InputPort<std::string>("v_pitch"),
        InputPort<std::string>("v_yaw")
    };
}

/**
 * Executes the command
 */
NodeStatus big_move_state::tick() {
    // ros::AsyncSpinner spinner(1);
    // spinner.start();

    ROS_INFO("Entering Move State.");
    
    while(ros::ok()) {
        ros::spinOnce();

        //run onEnter if this is the first time this command is being ticked
        if(!hasEntered) { 
            hasEntered = onEnter();
            continue; //this avoids checking if the command is finished if we havent entered yethasEntered = onEnter();
        }

        publishGoalPose();

        //the controller is currently handling movement (was stated in onEnter()).
        //here, we basically check to see if the position is acheived.
        //when the robot is steady within the target threshold, command ends
        if(locExists) { //if latest loc message exists...
            geometry_msgs::Point position = latestLocData.position;

            geometry_msgs::Quaternion orientation = latestLocData.orientation;

            bool
                positionGood = true,
                orientationGood = true;

            if(publishPosition) { //check position if changing it
                positionGood = abs(position.x - goal.position.x) < threshold
                            && abs(position.y - goal.position.y) < threshold
                            && abs(position.z - goal.position.z) < threshold;
            }

            if(publishOrientation) {
                geometry_msgs::Vector3
                    goalRPY = toRPY(goal.orientation),
                    currentRPY = toRPY(orientation);
                
                orientationGood = abs(goalRPY.x - currentRPY.x) < threshold
                               && abs(goalRPY.y - currentRPY.y) < threshold
                               && abs(goalRPY.z - currentRPY.z) < threshold;
            }

            // ROS_INFO("Orientation: %f, %f, %f, %f", orientation.x, orientation.y, orientation.z, orientation.w);

            if(positionGood && orientationGood) { //if it's steady then the orientation will be good as well
                ROS_INFO("Move finished!");

                if(publishAngVelocity) {
                    ros::Duration(1).sleep();
                    
                    geometry_msgs::Vector3 stopSpinVector;
                    angVelocityPublisher.publish(stopSpinVector);
                }

                return NodeStatus::SUCCESS;
            }
        }
    }

    return NodeStatus::FAILURE; //ros died, so command failed.
}


/**
 * Called just before the first tick is run
 * Returns true if successful, false otherwise.
 */
bool big_move_state::onEnter() {
    //wait for the robot to be steady
    // if(!steady) {
    //     return false;
    // }

    //wait for odometry data to exist
    if(!locExists) {
        return false;
    }
    
    publishPosition = true;
    publishOrientation = true;
    publishAngVelocity = true;

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
        ow = getInput<std::string>("orientation_w"),
        rv = getInput<std::string>("v_roll"),
        pv = getInput<std::string>("v_pitch"),
        yv = getInput<std::string>("v_yaw");

    //assign position data to goal pose
    if(x.value() != "" && y.value() != "" && z.value() != "") { //position fully defined, set it as goal position
        double
            xValue = std::stod(x.value()),
            yValue = std::stod(y.value()),
            zValue = std::stod(z.value());

        goal.position.x = xValue;
        goal.position.y = yValue;
        goal.position.z = zValue;
    } else { // position not fully defined, fall back to current position
        ROS_INFO("Goal Position not fully specified. Not publishing.");
        goal.position = currentPose.position;
        publishPosition = false;
    }

    //assign orientation data to goal pose
    if(ox.value() != "" && oy.value() != "" && oz.value() != "" && ow.value() != "") { //orientation fully defined
        double 
            oxValue = std::stod(ox.value()),
            oyValue = std::stod(oy.value()),
            ozValue = std::stod(oz.value()),
            owValue = std::stod(ow.value());

        goal.orientation.x = oxValue;
        goal.orientation.y = oyValue;
        goal.orientation.z = ozValue;
        goal.orientation.w = owValue;
    } else { //orientation not fully defined, fall back to current orientation
        ROS_INFO("Goal Orientation not fully specified. Not publishing.");
        goal.orientation = currentPose.orientation;
        publishOrientation = false;
    }

    //assign angular velocity data
    if(rv.value() != "" && pv.value() != "" && yv.value() != "") {
        double
            rvValue = std::stod(rv.value()),
            pvValue = std::stod(pv.value()),
            yvValue = std::stod(yv.value());

        //set angular velocity
        angVelocity.x = rvValue;
        angVelocity.y = pvValue;
        angVelocity.z = yvValue;

        //angular velocity will override the orientation controller, notify user here
        if(publishOrientation) {
            ROS_INFO("Angular velocity controller cannot be used with orientation controller. Disabling orientation.");
            publishOrientation = false;
        }
    } else {
        publishAngVelocity = false;
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

    publishGoalPose();
    ros::Duration(1).sleep();
    return true;
}

/**
 * publishes the goal position and orientation.
 */
void big_move_state::publishGoalPose() {
    //commands has entered, goal has been set.
    if(publishPosition) {
        positionPublisher.publish(goal.position);
    }

    if(publishOrientation) {
        orientationPublisher.publish(goal.orientation);
    }

    if(publishAngVelocity) {
        angVelocityPublisher.publish(angVelocity);
    }
}

/**
 * Converts a quaternion to rpy.
 */
geometry_msgs::Vector3 big_move_state::toRPY(geometry_msgs::Quaternion orientation) {
    tf2::Quaternion tf2Orientation;
    tf2::fromMsg(orientation, tf2Orientation);

    geometry_msgs::Vector3 rpy;
    tf2::Matrix3x3(tf2Orientation).getEulerYPR(rpy.z, rpy.y, rpy.x);
    return rpy;
}