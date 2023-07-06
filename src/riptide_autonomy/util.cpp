#include "riptide_autonomy/autonomy_lib.hpp"

using namespace std::chrono_literals;


std::string getEnvVar(const char *name)
{
    const char *env = std::getenv(name);
    if (env == nullptr)
    {
        RCLCPP_INFO(log, "DoTask: %s environment variable not found!", name);
        return "";
    }

    return std::string(env);
}


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
    tf2::doTransform(relative, result, transform);
    return result;
}

/**
 * @brief Transforms a relative pose between frames.
 *  
 * @param rosnode A ros node handle
 * @param buffer TF Buffer. THIS MUST BE ATTACHED TO A LISTENER THAT WAS CREATED WITH SPIN_THREAD = TRUE or else the function will not work
 * @param original Pose to tranform
 * @param fromFrame The frame original is currently in
 * @param toFrame The frame to transform original to
 * @param result The transformed pose
 * @return true if the operation succeeded, false otherwise
 */
bool transformBetweenFrames(rclcpp::Node::SharedPtr rosnode, std::shared_ptr<tf2_ros::Buffer> buffer, geometry_msgs::msg::Pose original, std::string fromFrame, std::string toFrame, geometry_msgs::msg::Pose& result) {    
    //look up transform with a three second timeout to find one
    rclcpp::Time startTime = rosnode->get_clock()->now();
    
    while((rosnode->get_clock()->now() - startTime) < 3s) {
        try {
            geometry_msgs::msg::TransformStamped transform = buffer->lookupTransform(toFrame, fromFrame, tf2::TimePointZero);
            result = doTransform(original, transform);

            return true;

        } catch(tf2::TransformException &ex) {
            RCLCPP_WARN_SKIPFIRST_THROTTLE(log, *rosnode->get_clock(), 500, "LookupException encountered while looking up transform from %s to %s.", fromFrame.c_str(), toFrame.c_str());
        }
    }
    RCLCPP_ERROR(log, "Failed to look up transform from %s to %s!", fromFrame.c_str(), toFrame.c_str());
    return false;
}


bool transformBetweenFrames(rclcpp::Node::SharedPtr rosnode, geometry_msgs::msg::Pose relative, std::string fromFrame, std::string toFrame, geometry_msgs::msg::Pose& result) {
    std::shared_ptr<tf2_ros::Buffer> buffer = std::make_shared<tf2_ros::Buffer>(rosnode->get_clock());
    tf2_ros::TransformListener listener(*buffer);

    return transformBetweenFrames(rosnode, buffer, relative, fromFrame, toFrame, result);
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
