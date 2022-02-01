#include "autonomy.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr n = std::make_shared<rclcpp::Node>("doTransform test");

    geometry_msgs::msg::Pose relative;

    relative.position.x = 1;
    relative.position.y = -1;
    relative.position.z = -2;
    relative.orientation.w = 1;
    relative.orientation.x = 0;
    relative.orientation.y = 0;
    relative.orientation.z = -1;

    tf2_ros::BufferClient buffer(n, "tf2_buffer_server");
    buffer.waitForServer();
    geometry_msgs::msg::TransformStamped transform = buffer.lookupTransform("world", "test", tf2::TimePointZero, tf2::durationFromSec(1.0));
    geometry_msgs::msg::Pose world = doTransform(relative, transform);

    RCLCPP_INFO(log, "World Pose: %f, %f, %f with orientation %f %f %f %f",
        world.position.x,
        world.position.y,
        world.position.z,
        world.orientation.w,
        world.orientation.x,
        world.orientation.y,
        world.orientation.z    
    );
}