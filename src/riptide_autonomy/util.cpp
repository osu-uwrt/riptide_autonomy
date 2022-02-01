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
    result.orientation = tf2::toMsg(resultQuat);

    return result;
}