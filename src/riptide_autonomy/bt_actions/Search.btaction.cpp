#include "bt_actions/Search.h"

using namespace BT;
using std::placeholders::_1;

//this is really annoying to type out...
typedef rclcpp::Publisher<riptide_msgs2::msg::ControllerCommand>::SharedPtr ControllerPublisher;

geometry_msgs::msg::Pose 
    guessLocation,
    currentPose;

double 
    guessError;

bool
    receivedGuess,
    receivedPos;


PortsList Search::providedPorts() {
    return {
        BT::InputPort<std::string>("frame"),
        BT::InputPort<std::string>("target_error"),
        BT::InputPort<std::string>("update_sec")
    };
}


static void guessCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) { 
    guessLocation = msg->pose.pose;
    guessError = msg->pose.covariance[0] + msg->pose.covariance[7] + msg->pose.covariance[14] + msg->pose.covariance[21] + msg->pose.covariance[28] + msg->pose.covariance[35];
    receivedGuess = true;
}


static void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) { 
    currentPose = msg->pose.pose;
    receivedPos = true;
}


static void publishPosition(ControllerPublisher positionPub, geometry_msgs::msg::Vector3 pos) {
    riptide_msgs2::msg::ControllerCommand cmd;
    cmd.mode = riptide_msgs2::msg::ControllerCommand::POSITION;
    cmd.setpoint_vect = pos;
    positionPub->publish(cmd);
}


static void publishOrientation(ControllerPublisher orientationPub, geometry_msgs::msg::Quaternion quat) {
    riptide_msgs2::msg::ControllerCommand cmd;
    cmd.mode = riptide_msgs2::msg::ControllerCommand::POSITION;
    cmd.setpoint_quat = quat;
    orientationPub->publish(cmd);
}


NodeStatus Search::tick() {
    auto odomSub        = rosnode->create_subscription<nav_msgs::msg::Odometry>(ODOMETRY_TOPIC, 10, std::bind(&odomCallback, _1));
    auto positionPub    = rosnode->create_publisher<riptide_msgs2::msg::ControllerCommand>(POSITION_TOPIC, 10);
    auto orientationPub = rosnode->create_publisher<riptide_msgs2::msg::ControllerCommand>(ORIENTATION_TOPIC, 10);

    RCLCPP_INFO(log, "BIG Search");
    bool sunnyDay = true;

    std::string target = getInput<std::string>("frame").value();
    std::string frame = target + "_frame";
    double target_error = stod(getInput<std::string>("target_error").value());
    double update_sec = stod(getInput<std::string>("update_sec").value());

    //Subscribe to the guess place (we initialize this here because we only now have read the behaviortree and know the frame name)
    auto guessSub = rosnode->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("mapping/" + target, 10, std::bind(&guessCallback, _1));

    //actually look up the transform of the frame we want
    tf2_ros::Buffer buffer(rosnode->get_clock());
    tf2_ros::TransformListener tfListener(buffer);
    geometry_msgs::msg::TransformStamped transform;
    RCLCPP_INFO(log, "Doing transform lookup");
    while(rclcpp::ok()) {
        try {
            transform = buffer.lookupTransform("world", frame, tf2::TimePointZero);
            break; 
        } catch(tf2::TransformException& ex) {
            RCLCPP_DEBUG(log, "%s", ex.what());
            // rclcpp::spin_some(rosnode);
        }
    }

    RCLCPP_INFO(log, "Lookup Complete");

    geometry_msgs::msg::Pose relativePose; //Pose in the frame of the object to find

    //Get one foot infront of the object, facing it
    relativePose.position.x = 1;
    relativePose.position.y = 0;
    relativePose.position.z = 0;
    relativePose.orientation.x = 0;
    relativePose.orientation.y = 0;
    relativePose.orientation.z = 1;
    relativePose.orientation.w = 0;

    geometry_msgs::msg::Pose worldGuessPos = doTransform(relativePose, transform);

    geometry_msgs::msg::Vector3 worldGuessPtv3;
    worldGuessPtv3.x = worldGuessPos.position.x;
    worldGuessPtv3.y = worldGuessPos.position.y;
    worldGuessPtv3.z = worldGuessPos.position.z;

    //publish the initial estimate to the controller to move the robot to that pose
    publishPosition(positionPub, worldGuessPtv3);
    publishOrientation(orientationPub, worldGuessPos.orientation);

    RCLCPP_INFO(log, "Going to %f, %f, %f", worldGuessPtv3.x, worldGuessPtv3.y, worldGuessPtv3.z);
    
    //Get a snapshot of the time at the start
    rclcpp::Time begin = rosnode->now();

    while(sunnyDay){
        if(rosnode->now() - begin >= rclcpp::Duration::from_seconds(update_sec)){
            //Get '1 foot infront of object' translated into the new frame of the new guess
            RCLCPP_INFO(log, "relative pos: %f, %f, %f. Transform: %f, %f, %f", relativePose.position.x, relativePose.position.y, relativePose.position.z, transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
            worldGuessPos = doTransform(relativePose, transform);

            RCLCPP_INFO(log, "Current Error: %f", guessError);

            if(guessError <= target_error){ //Target_error is updated by the subscriber callback automatically
                RCLCPP_INFO(log, "Search Finihsed!");
                return NodeStatus::SUCCESS;
            } else {
                
                worldGuessPtv3.x = worldGuessPos.position.x;
                worldGuessPtv3.y = worldGuessPos.position.y;
                worldGuessPtv3.z = worldGuessPos.position.z;
                publishPosition(positionPub, worldGuessPtv3);
                
                //figure out which direction to face object we are driving towards
                double
                    dx = worldGuessPtv3.x - currentPose.position.x,
                    dy = worldGuessPtv3.y - currentPose.position.y,
                    yaw = atan2(dy, dx);
                
                geometry_msgs::msg::Vector3 rpy;
                rpy.x = 0;
                rpy.y = 0;
                rpy.z = yaw;

                geometry_msgs::msg::Quaternion quat = toQuat(rpy);
                publishOrientation(orientationPub, quat);

                RCLCPP_INFO(log, "Going to %f, %f, %f", worldGuessPtv3.x, worldGuessPtv3.y, worldGuessPtv3.z);
            }

            //If the distance from the robot to the new location is < 1ft and the error is not low enough, sunnyDay = false;
            if(receivedPos && distance(vector3ToPoint(worldGuessPtv3), relativePose.position) <= 1 && guessError>target_error){
                sunnyDay = false;
            }

            begin = rosnode->now();
        }
    }

    RCLCPP_ERROR(log, "Search reached target object but error was still %f!", guessError);
    return NodeStatus::FAILURE;
}

