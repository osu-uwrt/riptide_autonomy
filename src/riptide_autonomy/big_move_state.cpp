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
    this->steadySubscriber = n.subscribe(steadyTopic, CACHE_SIZE, &states::big_move_state::steadyCallback, this);
    this->locSubscriber    = n.subscribe(locTopic, CACHE_SIZE, &states::big_move_state::locCallback, this);
    
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
            hasEntered = onEnter();
        }

        //moveit is currently handling movement (was stated in onEnter()).
        //here, we basically check to see if the position is acheived.
        //when the robot is steady within the target threshold, command ends
        if(latestLocMessage != 0) { //if latest loc message exists...
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
                break; 
            }
        } else {
            ROS_ERROR("Localization Message %s not there!", locTopic.c_str());
        }
    }

    ROS_INFO("command finished!");
    return NodeStatus::SUCCESS; // command finishes!
}


/**
 * Called just before the first tick is run
 * Returns true if successful, false otherwise.
 */
bool big_move_state::onEnter() {
    ros::AsyncSpinner spinner(1);
    spinner.start();
  
    //rate at which errors print (to avoid wall of red text)
    ros::Rate errPrintRate(2); //2 hz

    //wait for the position to be steady
    // bool steady = false;
    // while(!steady) {
    //     if(steady != 0) { //if steady message exists...
    //         steady = latestSteadyMessage->data;
    //     } else {
    //         ROS_WARN("Steady Message %s not there!", steadyTopic.c_str());
    //         errPrintRate.sleep();
    //     }
    // }

    //assign goal odometry data
    x = getInput<double>("x").value();
    y = getInput<double>("y").value();
    z = getInput<double>("z").value();
    orientation = Util::quaternionFromString(getInput<std::string>("orientation").value());

    ROS_INFO("Moving to position %f, %f, %f with orientation %s", x, y, z, getInput<std::string>("orientation").value().c_str());

    //create path
    moveit::planning_interface::MoveGroupInterface moveGroup(groupName);
    
    //define a JointModelGroup for performance improvement
    const robot_state::JointModelGroup* jointModelGroup = 
        moveGroup.getCurrentState()->getJointModelGroup(groupName);
    
    // //define goal position and orientation
    geometry_msgs::PoseStamped jointGoal;
    jointGoal.header.frame_id = groupName;
    jointGoal.pose.orientation.w = 1;
    jointGoal.pose.position.x = 0;
    jointGoal.pose.position.y = 0;
    jointGoal.pose.position.z = 0;

    moveGroup.setPlanningTime(1); //1 second to plan

    //define bounds of movement?
    moveGroup.setWorkspace(-50, -50, -30, 50, 50, 0);

    //plan the path
    // moveit::planning_interface::MoveGroupInterface::Plan movePlan;
    // moveGroup.setPoseTarget(jointGoal);

    // moveit::planning_interface::MoveItErrorCode planResult = moveGroup.plan(movePlan);
    // if(planResult == moveit::planning_interface::MoveItErrorCode::FAILURE) {
    //     ROS_ERROR("Path Planning Failed!");
    //     return false;
    // }

    //make the robot move
    // moveGroup.move();

    auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(n);
    auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(groupName, moveit_cpp_ptr);
    auto start_state = planning_components->getStartState();
    planning_components->setStartStateToCurrentState();
    planning_components->setGoal(jointGoal, groupName);

    auto planSolution = planning_components->plan();
    if(planSolution) {
        planning_components->execute();
    }
    return true;
}
