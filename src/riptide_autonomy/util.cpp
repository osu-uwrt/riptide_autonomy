#include "autonomy.h"

geometry_msgs::msg::Pose doTransform(geometry_msgs::msg::Pose relative, geometry_msgs::msg::TransformStamped transform) {
    geometry_msgs::msg::Pose result;
    
    geometry_msgs::msg::Vector3 translation = transform.transform.translation;
    result.position.x = relative.position.x + translation.x;
    result.position.y = relative.position.y + translation.y;
    result.position.z = relative.position.z + translation.z;

    tf2::Quaternion transformQuat;
    tf2::fromMsg(transform.transform.rotation, transformQuat);

    tf2::Quaternion relativeQuat;
    tf2::fromMsg(relative.orientation, relativeQuat);

    tf2::Quaternion resultQuat = relativeQuat * transformQuat;
    resultQuat.normalize();
    result.orientation = tf2::toMsg(resultQuat);

    return result;
}

geometry_msgs::msg::Vector3 toRPY(geometry_msgs::msg::Quaternion orientation) {
    tf2::Quaternion tf2Orientation;
    tf2::fromMsg(orientation, tf2Orientation);

    geometry_msgs::msg::Vector3 rpy;
    tf2::Matrix3x3(tf2Orientation).getEulerYPR(rpy.z, rpy.y, rpy.x);
    return rpy;
}

double distance(geometry_msgs::msg::Point point1, geometry_msgs::msg::Point point2){
    return sqrt(pow(point2.x - point1.x, 2) +pow(point2.y - point1.y, 2) + pow(point2.y - point1.y, 2));
}