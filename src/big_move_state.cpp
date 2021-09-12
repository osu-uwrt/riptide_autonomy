#include "states.h"

/**
 * Source file for the big_move_state class.
 * Written By: Brach Knutson
 */

using namespace states;
using namespace BT;

void big_move_state::steadyCallback(const std_msgs::Bool::ConstPtr& msg) {
    latestSteadyMessage = msg;
}

void big_move_state::locCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    latestLocMessage = msg;
}

/**
 * Basically a constructor
 */
void big_move_state::init() {
    //create cached subscriber
    ros::NodeHandle n;
    this->steadySubscriber = n.subscribe(steadyTopic, CACHE_SIZE, steadyCallback, this);
    this->locSubscriber    = n.subscribe(locTopic, CACHE_SIZE, locCallback, this);
    
    hasEntered = false;
}

/**
 * Defines needed ports
 */
PortsList big_move_state::providedPorts() {
    return { 
        InputPort<double>("x"),
        InputPort<double>("y"),
        InputPort<double>("z"),
        InputPort<geometry_msgs::Quaternion>("orientation")
    };
}

/**
 * Executes the command
 */
NodeStatus big_move_state::tick() {
    //run onEnter if this is the first time this command is being ticked
    if(!hasEntered) {
        hasEntered = true;
        bool enterSuccess = onEnter();
        
        if(!enterSuccess) {
            return NodeStatus::FAILURE;
        }
    }

    //moveit is currently handling movement (was stated in onEnter()).
    //here, we basically check to see if the position is acheived.
    //when the robot is steady within the target threshold, command ends
    nav_msgs::Odometry::ConstPtr pos = latestLocMessage;
    geometry_msgs::Point position = pos->pose.pose.position;
    double
        userX = getInput<double>("x").value(),
        userY = getInput<double>("y").value(),
        userZ = getInput<double>("z").value();

    bool
        xGood = abs(position.x - userX) < threshold,
        yGood = abs(position.y - userY) < threshold,
        zGood = abs(position.z - userZ) < threshold,
        isSteady = latestSteadyMessage->data;

    if(xGood && yGood && zGood && isSteady) {
        return NodeStatus::SUCCESS; //Command Finishes!
    }
    

    return NodeStatus::RUNNING; //Command still going
}

/**
 * Called just before the first tick is run
 * Returns true if successful, false otherwise.
 */
bool big_move_state::onEnter() {
    //wait for the position to be steady
    bool steady = false;
    while(!steady) {
        steady = latestSteadyMessage->data;
    }

    //assign steady odometry data
    x = getInput<double>("x").value();
    y = getInput<double>("y").value();
    z = getInput<double>("z").value();
    orientation = getInput<geometry_msgs::Quaternion>("orientation").value();
    
    //skipped the if statement for null orientation because c++ does
    //not have "null." Orientation must be defined, even if it is zeros.

    //create path
    moveit::planning_interface::MoveGroupInterface moveGroup(groupName);
    moveit::planning_interface::PlanningSceneInterface planningSceneInterface;
    
    //define a JointModelGroup for performance improvement
    const robot_state::JointModelGroup* jointModelGroup = 
        moveGroup.getCurrentState()->getJointModelGroup(groupName);
    
    //define goal position and orientation
    geometry_msgs::Pose jointGoal;
    jointGoal.orientation = orientation;
    jointGoal.position.x = x;
    jointGoal.position.y = y;
    jointGoal.position.z = z;

    moveGroup.setPlanningTime(1); //1 second to plan

    //define bounds of movement?
    moveGroup.setWorkspace(-50, -50, -30, 50, 50, 0);

    //plan the path
    moveit::planning_interface::MoveGroupInterface::Plan movePlan;
    moveit::planning_interface::MoveItErrorCode planResult = moveGroup.plan(movePlan);
    if(planResult == moveit::planning_interface::MoveItErrorCode::FAILURE) {
        return false;
    }

    moveGroup.execute(movePlan);
    return true;
}