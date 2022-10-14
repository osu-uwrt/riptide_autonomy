#include "riptide_autonomy/autonomy_lib.hpp"


void registerPluginsForFactory(std::shared_ptr<BT::BehaviorTreeFactory> factory, const std::string packageName) {
    std::string amentIndexPath = ament_index_cpp::get_package_prefix(packageName); // TODO Make this work to scan ament index and get to our plugin
    factory->registerFromPlugin(amentIndexPath + "/lib/libautonomy_actions.so");
    factory->registerFromPlugin(amentIndexPath + "/lib/libautonomy_conditions.so");
    factory->registerFromPlugin(amentIndexPath + "/lib/libautonomy_decorators.so");
}


void initRosForTree(BT::Tree& tree, rclcpp::Node::SharedPtr rosContext) {
    // give each BT node access to our RCLCPP context
    for (auto &node : tree.nodes)
    {
        // Not a typo: it is "=", not "=="
        if (auto btNode = dynamic_cast<UwrtBtNode *>(node.get()))
        {
            btNode->init(rosContext);
        }
    }
}


geometry_msgs::msg::Pose doTransform(geometry_msgs::msg::Pose relative, geometry_msgs::msg::TransformStamped transform) {
    geometry_msgs::msg::Pose result;

    //rotate the position on the yaw based on the transform rotation to the object
    double yaw = toRPY(transform.transform.rotation).z;
    
    geometry_msgs::msg::Vector3 newRelative;
    newRelative.x = relative.position.x * cos(yaw) - relative.position.y * sin(yaw);
    newRelative.y = relative.position.x * sin(yaw) + relative.position.y * cos(yaw);
    
    //translate point to new frame
    geometry_msgs::msg::Vector3 translation = transform.transform.translation;
    result.position.x = newRelative.x + translation.x;
    result.position.y = newRelative.y + translation.y;
    result.position.z = relative.position.z + translation.z;

    //transform the orientation quaternions
    tf2::Quaternion transformQuat;
    tf2::fromMsg(transform.transform.rotation, transformQuat);

    tf2::Quaternion relativeQuat;
    tf2::fromMsg(relative.orientation, relativeQuat);

    tf2::Quaternion resultQuat = relativeQuat * transformQuat;
    resultQuat.normalize();
    result.orientation = tf2::toMsg(resultQuat);

    return result;
}

std::tuple<geometry_msgs::msg::Pose, bool> transformBetweenFrames(geometry_msgs::msg::Pose original, std::string fromFrame, std::string toFrame, rclcpp::Node::SharedPtr rosnode, std::shared_ptr<tf2_ros::Buffer> buffer) {
    bool printedWarning = false;
    //look up transform with a three second timeout to find one
    rclcpp::Time startTime = rosnode->get_clock()->now();
    while((rosnode->get_clock()->now() - startTime).seconds() < 1) {
        try {
            geometry_msgs::msg::TransformStamped transform = buffer->lookupTransform(toFrame, fromFrame, tf2::TimePointZero);
            geometry_msgs::msg::Pose transformed = doTransform(original, transform);

            RCLCPP_INFO(log, "transformed position: %f, %f, %f", transformed.position.x, transformed.position.y, transformed.position.z);
            RCLCPP_INFO(log, "transformed orientation: %f, %f, %f, %f", transformed.orientation.x, transformed.orientation.y, transformed.orientation.z, transformed.orientation.w);

            return std::make_tuple(transformed, true);

        } catch(tf2::LookupException &ex) {
            if(!printedWarning) {
                RCLCPP_WARN(log, "LookupException encountered while looking up transform from %s to %s.", fromFrame.c_str(), toFrame.c_str());
                printedWarning = true;
            }
        }
    }
    RCLCPP_ERROR(log, "Failed to look up transform from %s to %s!", fromFrame.c_str(), toFrame.c_str());
    geometry_msgs::msg::Pose failed; 
    return std::make_tuple(failed, false);
}


std::tuple<geometry_msgs::msg::Pose, bool> transformBetweenFrames(geometry_msgs::msg::Pose relative, std::string fromFrame, std::string toFrame, rclcpp::Node::SharedPtr rosnode) {
    std::shared_ptr<tf2_ros::Buffer> buffer = std::make_shared<tf2_ros::Buffer>(rosnode->get_clock());
    tf2_ros::TransformListener listener(*buffer);

    return transformBetweenFrames(relative, toFrame, fromFrame, rosnode, buffer);
}


geometry_msgs::msg::Vector3 pointToVector3(geometry_msgs::msg::Point pt) {
    geometry_msgs::msg::Vector3 v;
    v.x = pt.x;
    v.y = pt.y;
    v.z = pt.z;
    return v;
}

geometry_msgs::msg::Point vector3ToPoint(geometry_msgs::msg::Vector3 v) {
    geometry_msgs::msg::Point pt;
    pt.x = v.x;
    pt.y = v.y;
    pt.z = v.z;
    return pt;
}

double vector3Length(geometry_msgs::msg::Vector3 v) {
    return sqrt((v.x * v.x) + (v.y * v.y) + (v.z * v.z));
}

geometry_msgs::msg::Vector3 toRPY(geometry_msgs::msg::Quaternion orientation) {
    tf2::Quaternion tf2Orientation;
    tf2::fromMsg(orientation, tf2Orientation);

    geometry_msgs::msg::Vector3 rpy;
    tf2::Matrix3x3(tf2Orientation).getEulerYPR(rpy.z, rpy.y, rpy.x);
    return rpy;
}

geometry_msgs::msg::Quaternion toQuat(geometry_msgs::msg::Vector3 rpy) {
    tf2::Quaternion tf2Quat;
    tf2Quat.setRPY(rpy.x, rpy.y, rpy.z);
    tf2Quat.normalize();

    return tf2::toMsg(tf2Quat);
}

double distance(geometry_msgs::msg::Point point1, geometry_msgs::msg::Point point2) {
    return sqrt(pow(point2.x - point1.x, 2) +pow(point2.y - point1.y, 2) + pow(point2.z - point1.z, 2));
}

double distance(geometry_msgs::msg::Vector3 point1, geometry_msgs::msg::Vector3 point2) {
    return distance(vector3ToPoint(point1), vector3ToPoint(point2));
}

std::string stringWithBlackboardEntries(std::string str, BT::TreeNode& btNode) {
    std::string result = "";
    int pos = 0;
    while(str.find_first_of('{', pos) != std::string::npos) {
        int lbpos = str.find_first_of("{", pos);
        result += str.substr(pos, lbpos - pos); //add everything from pos up until the "{" to result

        if(str.find_first_of("}", lbpos) != std::string::npos) {
            int rbpos = str.find_first_of("}", lbpos);

            std::string 
                tokenWithBrackets = str.substr(lbpos, rbpos - lbpos + 1),
                nameOfEntry = tokenWithBrackets.substr(1, tokenWithBrackets.length() - 2),
                valueOfEntry;

            //get the value of the entry
            if(getFromBlackboard<std::string>(btNode, nameOfEntry, valueOfEntry)) {
                //if nameOfEntry exists, valueOfEntry was populated by the call above
                result += valueOfEntry;
            } else {
                result += tokenWithBrackets; //put whole token in because it didn't lead anywhere
            }

            pos = rbpos + 1; //set position to after '}'
        } else {
            pos = lbpos + 1; //set position to after '{ (there is no '}')
        }
    }

    result += str.substr(pos);
    return result;
}
