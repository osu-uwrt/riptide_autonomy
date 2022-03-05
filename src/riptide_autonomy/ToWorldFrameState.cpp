#include "autonomy.h"

using namespace BT;


void ToWorldFrameState::init(rclcpp::Node::SharedPtr node) {
    rosnode = node;
}


NodeStatus ToWorldFrameState::tick() {
    //start buffer client to look up transform
    tf2_ros::Buffer buffer(rosnode->get_clock());
    tf2_ros::TransformListener listener(buffer);

    //look up object (should be broadcasted from mapping)
    std::string objectName = getInput<std::string>("object").value();
    geometry_msgs::msg::TransformStamped transform;
    while(rclcpp::ok()) {
        try {
            transform = buffer.lookupTransform("world", objectName, tf2::TimePointZero);
            break; 
        } catch(tf2::TransformException& ex) {
            RCLCPP_DEBUG(log, "%s", ex.what());
            rclcpp::spin_some(rosnode);
        }
    }

    geometry_msgs::msg::Pose relativePose;
    
    //get current pose to translate
    relativePose.position.x = std::stod(getInput<std::string>("relative_x").value());
    relativePose.position.y = std::stod(getInput<std::string>("relative_y").value());
    relativePose.position.z = std::stod(getInput<std::string>("relative_z").value());
    relativePose.orientation.x = std::stod(getInput<std::string>("relative_orientation_x").value());
    relativePose.orientation.y = std::stod(getInput<std::string>("relative_orientation_y").value());
    relativePose.orientation.z = std::stod(getInput<std::string>("relative_orientation_z").value());
    relativePose.orientation.w = std::stod(getInput<std::string>("relative_orientation_w").value());

    //transform the current pose into desired
    geometry_msgs::msg::Pose desiredPose = doTransform(relativePose, transform);

    //output desired
    setOutput<std::string>("world_x", std::to_string(desiredPose.position.x));
    setOutput<std::string>("world_y", std::to_string(desiredPose.position.y));
    setOutput<std::string>("world_z", std::to_string(desiredPose.position.z));
    setOutput<std::string>("world_orientation_x", std::to_string(desiredPose.orientation.x));
    setOutput<std::string>("world_orientation_y", std::to_string(desiredPose.orientation.y));
    setOutput<std::string>("world_orientation_z", std::to_string(desiredPose.orientation.z));
    setOutput<std::string>("world_orientation_w", std::to_string(desiredPose.orientation.w));

    RCLCPP_INFO(log, "World Frame Calculated.");

    return NodeStatus::SUCCESS;
}

