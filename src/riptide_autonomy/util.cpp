#include "autonomy.h"


geometry_msgs::msg::Pose doTransform(geometry_msgs::msg::Pose relative, geometry_msgs::msg::TransformStamped transform) {
    geometry_msgs::msg::Pose result;

    //rotate the position based on the transform rotation to the object
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

geometry_msgs::msg::Vector3 toRPY(geometry_msgs::msg::Quaternion orientation) {
    tf2::Quaternion tf2Orientation;
    tf2::fromMsg(orientation, tf2Orientation);

    geometry_msgs::msg::Vector3 rpy;
    tf2::Matrix3x3(tf2Orientation).getEulerYPR(rpy.z, rpy.y, rpy.x);
    return rpy;
}

double distance(geometry_msgs::msg::Point point1, geometry_msgs::msg::Point point2) {
    return sqrt(pow(point2.x - point1.x, 2) +pow(point2.y - point1.y, 2) + pow(point2.y - point1.y, 2));
}

double distance(geometry_msgs::msg::Vector3 point1, geometry_msgs::msg::Vector3 point2) {
    return distance(vector3ToPoint(point1), vector3ToPoint(point2));
}

template<typename T>
bool getFromBlackboard(BT::TreeNode& n, std::string key, T *value) {
    try {
        if(n.config().blackboard.get()->get<T>(key, *value)) {
            return true;
        }
    } catch (std::runtime_error const&) {
        RCLCPP_ERROR(log, "Could not cast blackboard entry \"%s\" to the correct type.", key.c_str());
    }

    return false;
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
            if(getFromBlackboard<std::string>(btNode, nameOfEntry, &valueOfEntry)) {
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
