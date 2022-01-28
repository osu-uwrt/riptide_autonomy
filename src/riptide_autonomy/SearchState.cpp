#include "autonomy.h"

using namespace BT;
using namespace std::placeholders;

    
void SearchState::init(rclcpp::Node::SharedPtr node) {
    rosnode = node;

    odomSub        = node->create_subscription<nav_msgs::msg::Odometry>(ODOMETRY_TOPIC, 10, std::bind(&SearchState::odomCallback, this, _1));
    positionPub    = node->create_publisher<geometry_msgs::msg::Point>(POSITION_TOPIC, 10);
    orientationPub = node->create_publisher<geometry_msgs::msg::Quaternion>(ORIENTATION_TOPIC, 10);
}


NodeStatus SearchState::tick() {
    RCLCPP_INFO(log, "BIG Search");
    bool sunnyDay = true;

    std::string target = getInput<std::string>("frame").value();
    std::string frame = target + "_frame";
    double target_error = stod(getInput<std::string>("target_error").value());
    double update_sec = stod(getInput<std::string>("update_sec").value());

    //Subscribe to the guess place (we initialize this here because we only now have read the behaviortree and know the frame name)
    guessSub = rosnode->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("mapping/" + target, 10, std::bind(&SearchState::guessCallback, this, _1));

    //create buffer server to look for frame transform
    RCLCPP_INFO(log, "Starting TF2 Buffer Server");
    tf2_ros::BufferClient buffer(rosnode, "tf2_buffer_server");
    buffer.waitForServer();
    RCLCPP_INFO(log, "TF2 Buffer Server connected.");

    //actually look up the transform of the frame we want
    geometry_msgs::msg::TransformStamped transform;
    RCLCPP_INFO(log, "Doing transform lookup");
    while(rclcpp::ok()) {
        try {
            transform = buffer.lookupTransform("world",frame, tf2::TimePointZero, tf2::durationFromSec(1.0));
            break; 
        } catch(tf2::TransformException ex) {
            RCLCPP_WARN(log, "%s", ex.what());
        }
    }

    RCLCPP_INFO(log, "Lookup Complete");

    geometry_msgs::msg::Pose currentPose; //Pose in the frame of the object to find

    //Get one foot infront of the object, facing it
    currentPose.position.x = 0;
    currentPose.position.y = 0;
    currentPose.position.z = 0;
    currentPose.orientation.x = 0;
    currentPose.orientation.y = 0;
    currentPose.orientation.z = 0;
    currentPose.orientation.w = 1;

    geometry_msgs::msg::Pose worldGuessPos;
    tf2::doTransform(currentPose, worldGuessPos, transform);

    geometry_msgs::msg::Point worldGuessPt;
    worldGuessPt.x = worldGuessPos.position.x;
    worldGuessPt.y = worldGuessPos.position.y;
    worldGuessPt.z = worldGuessPos.position.z;

    //publish the initial estimate to the controller to move the robot to that pose
    positionPub->publish(worldGuessPt);
    orientationPub->publish(worldGuessPos.orientation);

    RCLCPP_INFO(log, "Going to %f, %f, %f", worldGuessPt.x, worldGuessPt.y, worldGuessPt.z);
    
    //Get a snapshot of the time at the start
    rclcpp::Time begin = rosnode->now();

    while(sunnyDay){
        if(rosnode->now() - begin >= rclcpp::Duration::from_seconds(update_sec)){
            //Get '1 foot infront of object' translated into the new frame of the new guess
            tf2::doTransform(currentPose, worldGuessPos, transform);

            RCLCPP_INFO(log, "Current Error: %f", guessError);

            if(guessError <= target_error){ //Target_error is updated by the subscriber callback automatically
                RCLCPP_INFO(log, "Search Finihsed!");
                return NodeStatus::SUCCESS;
            } else {
                
                worldGuessPt.x = worldGuessPos.position.x;
                worldGuessPt.y = worldGuessPos.position.y;
                worldGuessPt.z = worldGuessPos.position.z;
                positionPub->publish(worldGuessPt); 
                orientationPub->publish(worldGuessPos.orientation);

                RCLCPP_INFO(log, "Going to %f, %f, %f", worldGuessPt.x, worldGuessPt.y, worldGuessPt.z);
            }

            //If the distance from the robot to the new location is < 1ft and the error is not low enough, sunnyDay = false;
            if(receivedPos && distance(worldGuessPt, currentPose.position)<=1 && guessError>target_error){
                sunnyDay = false;
            }

            begin = rosnode->now();
        }
    }

    return NodeStatus::FAILURE;
}


void SearchState::guessCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) { 
    guessLocation = msg->pose.pose;
    guessError = msg->pose.covariance[0] + msg->pose.covariance[7] + msg->pose.covariance[14] + msg->pose.covariance[21] + msg->pose.covariance[28] + msg->pose.covariance[35];
    receivedGuess = true;
}


void SearchState::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) { 
    currentPose = msg->pose.pose;
    receivedPos = true;
}


double SearchState::distance(geometry_msgs::msg::Point point1, geometry_msgs::msg::Point point2){
    return sqrt(pow(point2.x - point1.x, 2) +pow(point2.y - point1.y, 2) + pow(point2.y - point1.y, 2));
}
