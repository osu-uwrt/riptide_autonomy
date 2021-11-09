#include "states.h"

/**
 * Source file for the search state class.
 */

using namespace BT;
using namespace states;

geometry_msgs::Pose guessLocation;
double guessError;
geometry_msgs::Vector3 robotLoc;
bool recievedGuess = false;
bool recievedPos = false;

/**
 * Called by the constructor of the class when the object is constructed.
 * Use this method to create any other publishers or subscribers that you need.
 */
void search_state::init() {
    positionPublisher = n.advertise<geometry_msgs::Vector3>(positionTopic, CACHE_SIZE);
    orientationPublisher = n.advertise<geometry_msgs::Quaternion>(orientationTopic, CACHE_SIZE);
}

/**
 * Returns the list of ports that the node relies on.
 */
PortsList search_state::providedPorts() {
    return {
        InputPort<std::string>("frame"),
        InputPort<std::string>("target_error"),
        InputPort<std::string>("update_sec")
    };
}


void search_state::guessCallback(const geometry_msgs::PoseWithCovarianceStamped& msg) { //Not sure the message type
    guessLocation = msg.pose.pose;
    guessError = msg.pose.covariance[0]+msg.pose.covariance[7]+msg.pose.covariance[14]+msg.pose.covariance[21]+msg.pose.covariance[28]+msg.pose.covariance[35];
    recievedGuess = true;
}

void search_state::locCallback(const nav_msgs::Odometry::ConstPtr& msg) { //Not sure the message type
    robotLoc.x = msg -> pose.pose.position.x;
    robotLoc.y = msg -> pose.pose.position.y;
    robotLoc.z = msg -> pose.pose.position.z;
    recievedPos = true;
}

double search_state::distance(geometry_msgs::Vector3 point1, geometry_msgs::Vector3 point2){
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

    std::string target = getInput<std::string>("frame").value();
    std::string frame = target + "_frame";
    double target_error = stod(getInput<std::string>("target_error").value());
    double update_sec = stod(getInput<std::string>("update_sec").value());

    //Subscribe to the guess place
    guessSubscriber = n.subscribe("mapping/"+target, CACHE_SIZE, &states::search_state::guessCallback, this);
    //Robot position subscriber
    locSubscriber = n.subscribe(locTopic, CACHE_SIZE, &states::search_state::locCallback, this);


    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    geometry_msgs::TransformStamped transform = buffer.lookupTransform("world",frame, ros::Time(0), ros::Duration(1.0));
    geometry_msgs::Pose currentPose; //Pose in the frame of the object to find

    //Get one foot infront of the object, facing it
    currentPose.position.x = 1;
    currentPose.position.y = 0;
    currentPose.position.z = 0;
    currentPose.orientation.x = 0;
    currentPose.orientation.y = 0;
    currentPose.orientation.z = 1;
    currentPose.orientation.w = 0;

    geometry_msgs::Pose worldGuessPos;
    tf2::doTransform(currentPose, worldGuessPos, transform);
    geometry_msgs::Vector3 worldGuessV3;
    worldGuessV3.x = worldGuessPos.position.x;
    worldGuessV3.y = worldGuessPos.position.y;
    worldGuessV3.z = worldGuessPos.position.z;
    geometry_msgs::Vector3 posToMoveTo = worldGuessV3;
    
    //Get a snapshot of the time at the start
    ros::Time begin = ros::Time::now();

    while(sunnyDay){
        if(recievedGuess){
            if(ros::Time::now()-begin >=ros::Duration(update_sec)){
                //Get '1 foot infront of object' translated into the new frame of the new guess
                tf2::doTransform(currentPose, worldGuessPos, transform);

                if(guessError <= target_error){ //Target_error is updated by the subscriber callback automatically
                    return NodeStatus::SUCCESS;
                } else {
                    
                    worldGuessV3.x = worldGuessPos.position.x;
                    worldGuessV3.y = worldGuessPos.position.y;
                    worldGuessV3.z = worldGuessPos.position.z;
                    positionPublisher.publish(worldGuessV3); 
                    orientationPublisher.publish(worldGuessPos.orientation);

                }

                //If the distance from the robot to the new location is < 1ft and the error is not low enough, sunnyDay = false;
                if(recievedPos && distance(worldGuessV3, robotLoc)<=1 && guessError>target_error){
                    sunnyDay = false;
                }
            }

            

            begin = ros::Time::now();        
        
        }
    }


    return NodeStatus::FAILURE;
}