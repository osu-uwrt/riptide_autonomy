#include "riptide_autonomy/autonomy_lib.hpp"

using namespace std::chrono_literals;


std::string getEnvVar(const char *name)
{
    const char *env = std::getenv(name);
    if (!env)
    {
        throw std::invalid_argument(name);
    }

    return std::string(env);
}


void registerPluginsForFactory(std::shared_ptr<BT::BehaviorTreeFactory> factory, const std::string& packageName) {
    std::string amentIndexPath = ament_index_cpp::get_package_prefix(packageName); // TODO Make this work to scan ament index and get to our plugin
    factory->registerFromPlugin(amentIndexPath + "/lib/libautonomy_actions.so");
    factory->registerFromPlugin(amentIndexPath + "/lib/libautonomy_conditions.so");
    factory->registerFromPlugin(amentIndexPath + "/lib/libautonomy_decorators.so");
}


void initRosForTree(BT::Tree& tree, rclcpp::Node::SharedPtr rosNode) {
    //initialize static variables of UwrtBtNode
    UwrtBtNode::staticInit(rosNode);

    // give each BT node access to our RCLCPP context
    for (auto &treeNode : tree.nodes)
    {
        // Not a typo: it is "=", not "=="
        if (auto uwrtNode = dynamic_cast<UwrtBtNode *>(treeNode.get()))
        {
            uwrtNode->init(rosNode);
        }
    }
}


geometry_msgs::msg::Pose doTransform(geometry_msgs::msg::Pose relative, geometry_msgs::msg::TransformStamped transform) {
    geometry_msgs::msg::Pose result;
    tf2::doTransform(relative, result, transform);
    return result;
}


bool lookupTransformNow(
    rclcpp::Node::SharedPtr node,
    const std::shared_ptr<const tf2_ros::Buffer> buffer,
    const std::string& fromFrame,
    const std::string& toFrame,
    geometry_msgs::msg::TransformStamped& transform,
    bool lookupNext)
{
    try {
        tf2::TimePoint tp = (lookupNext ? tf2_ros::fromRclcpp(node->get_clock()->now()) : tf2::TimePointZero);
        transform = buffer->lookupTransform(toFrame, fromFrame, tp);
        return true;
    } catch(tf2::TransformException& ex) {
        RCLCPP_WARN(node->get_logger(), "Failed to look up transform from %s to %s (%s)", fromFrame.c_str(), toFrame.c_str(), ex.what());
    }
    
    return false;
}


bool lookupTransformThrottled(
    rclcpp::Node::SharedPtr node,
    const std::shared_ptr<const tf2_ros::Buffer> buffer,
    const std::string& fromFrame,
    const std::string& toFrame,
    double throttleDuration,
    double& lastLookup,
    geometry_msgs::msg::TransformStamped& transform,
    bool lookupNext)
{
    double 
        currentLookup = node->get_clock()->now().seconds(),
        elapsedSinceLastLookup = currentLookup - lastLookup;
    
    if(elapsedSinceLastLookup >= throttleDuration) {
        lastLookup = currentLookup;
        return lookupTransformNow(node, buffer, fromFrame, toFrame, transform, lookupNext);
    }
    return false; 
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


std::string formatStringWithBlackboard(const std::string& str, UwrtBtNode *n) {
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
            if(getFromBlackboard<std::string>(n, nameOfEntry, valueOfEntry)) {
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
