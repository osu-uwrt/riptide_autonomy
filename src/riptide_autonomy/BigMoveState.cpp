#include "autonomy.h"

using namespace BT;
using namespace std::placeholders;


void BigMoveState::init(rclcpp::Node::SharedPtr node) {
    this->rosnode = node;

    //create pubs and subs
    odomSubscriber = node->create_subscription<nav_msgs::msg::Odometry>(ODOMETRY_TOPIC, 10, std::bind(&BigMoveState::odomCallback, this, _1));

    positionPub = node->create_publisher<geometry_msgs::msg::Point>(POSITION_TOPIC, 10);
    orientationPub = node->create_publisher<geometry_msgs::msg::Quaternion>(ORIENTATION_TOPIC, 10);
    angVelocityPub = node->create_publisher<geometry_msgs::msg::Vector3>(ANGULAR_VELOCITY_TOPIC, 10);

    //initial values
    hasEntered = false;
    odomExists = false;
    publishPosition = false;
    publishOrientation = false;
    publishAngularVelocity = false;
}


NodeStatus BigMoveState::tick() {        
    RCLCPP_INFO(log, "Entering Move State.");

    resolveGoalPosition();

    while(rclcpp::ok()) {
        rclcpp::spin_some(rosnode);

        publishGoalInformation(); //"remind" the robot where it is going

        //the controller is currently handling movement (was stated in onEnter()).
        //here, we basically check to see if the position is acheived.
        //when the robot is steady within the target threshold, state ends
        if(odomExists) { //if latest loc message exists...
            geometry_msgs::msg::Point position = currentPose.position;
            geometry_msgs::msg::Quaternion orientation = currentPose.orientation;

            bool
                positionGood = true, //both default to true, will be updated below
                orientationGood = true;

            if(publishPosition) { //check position if setting it
                positionGood = abs(position.x - goalPose.position.x) < THRESHOLD
                            && abs(position.y - goalPose.position.y) < THRESHOLD
                            && abs(position.z - goalPose.position.z) < THRESHOLD;
            }

            if(publishOrientation) { //check orientation if setting it
                geometry_msgs::msg::Vector3
                    goalRPY = toRPY(goalPose.orientation), //use RPY because quaternions didn't work too well
                    currentRPY = toRPY(orientation);
                
                orientationGood = abs(goalRPY.x - currentRPY.x) < THRESHOLD
                                && abs(goalRPY.y - currentRPY.y) < THRESHOLD
                                && abs(goalRPY.z - currentRPY.z) < THRESHOLD;
            }

            if(positionGood && orientationGood) {
                RCLCPP_INFO(log, "Move state: Move finished!");

                if(publishAngularVelocity) {                        
                    geometry_msgs::msg::Vector3 stopSpinVector;
                    angVelocityPub->publish(stopSpinVector);
                }

                return NodeStatus::SUCCESS;
            }
        }
    }

    return NodeStatus::FAILURE; //rclcpp died, so command failed.
}


void BigMoveState::publishGoalInformation() {
    //commands has entered, goal has been set.
    if(publishPosition) {
        positionPub->publish(goalPose.position);
    }

    if(publishOrientation) {
        orientationPub->publish(goalPose.orientation);
    }

    if(publishAngularVelocity) {
        angVelocityPub->publish(goalAngVelocity);
    }
}


geometry_msgs::msg::Vector3 BigMoveState::toRPY(geometry_msgs::msg::Quaternion orientation) {
    tf2::Quaternion tf2Orientation;
    tf2::fromMsg(orientation, tf2Orientation);

    geometry_msgs::msg::Vector3 rpy;
    tf2::Matrix3x3(tf2Orientation).getEulerYPR(rpy.z, rpy.y, rpy.x);
    return rpy;
}


void BigMoveState::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    currentPose = msg->pose.pose;
    odomExists = true;
}


void BigMoveState::resolveGoalPosition() {
    //wait for odometry data to exist
    while(!odomExists) {
        rclcpp::spin_some(rosnode);
    }
    
    publishPosition = true;
    publishOrientation = true;
    publishAngularVelocity = true;

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

    //assign position data to goal pose if all ports have values
    if(x.value() != "" && y.value() != "" && z.value() != "") { //position fully defined, set it as goal position
        double
            xValue = std::stod(x.value()),
            yValue = std::stod(y.value()),
            zValue = std::stod(z.value());

        goalPose.position.x = xValue;
        goalPose.position.y = yValue;
        goalPose.position.z = zValue;
    } else { // position not fully defined, fall back to current position
        RCLCPP_INFO(log, "Move state: Goal Position not fully specified. Not publishing.");
        goalPose.position = currentPose.position;
        publishPosition = false;
    }

    //assign orientation data to goal pose if all ports have values
    if(ox.value() != "" && oy.value() != "" && oz.value() != "" && ow.value() != "") { //orientation fully defined
        double 
            oxValue = std::stod(ox.value()),
            oyValue = std::stod(oy.value()),
            ozValue = std::stod(oz.value()),
            owValue = std::stod(ow.value());

        goalPose.orientation.x = oxValue;
        goalPose.orientation.y = oyValue;
        goalPose.orientation.z = ozValue;
        goalPose.orientation.w = owValue;
    } else { //orientation not fully defined, fall back to current orientation
        RCLCPP_INFO(log, "Move state: Goal Orientation not fully specified. Not publishing.");
        goalPose.orientation = currentPose.orientation;
        publishOrientation = false;
    }

    //assign angular velocity data if all ports have values
    if(rv.value() != "" && pv.value() != "" && yv.value() != "") {
        double
            rvValue = std::stod(rv.value()),
            pvValue = std::stod(pv.value()),
            yvValue = std::stod(yv.value());

        //set angular velocity
        goalAngVelocity.x = rvValue;
        goalAngVelocity.y = pvValue;
        goalAngVelocity.z = yvValue;

        //angular velocity will override the orientation controller, notify user here
        if(publishOrientation) {
            RCLCPP_INFO(log, "Move state: Angular velocity controller cannot be used with orientation controller. Disabling orientation.");
            publishOrientation = false;
        }
    } else {
        publishAngularVelocity = false;
    }

    geometry_msgs::msg::Vector3 goalRPY = toRPY(goalPose.orientation);

    RCLCPP_INFO (log, 
        "Move state: Moving to position %f, %f, %f with orientation %f, %f, %f",
        goalPose.position.x,
        goalPose.position.y,
        goalPose.position.z,
        goalRPY.x,
        goalRPY.y,
        goalRPY.z
    );

    publishGoalInformation(); //publish initial goal position to robot
}  