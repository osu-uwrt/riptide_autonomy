#include "autonomy.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr n = std::make_shared<rclcpp::Node>("doTransformTest");

    //create tf buffer and broadcaster
    std::unique_ptr<tf2_ros::Buffer> buffer = std::make_unique<tf2_ros::Buffer>(n->get_clock());
    std::shared_ptr<tf2_ros::TransformListener> listener = std::make_shared<tf2_ros::TransformListener>(*buffer);

    //try to transform a position relative to our test frame
    geometry_msgs::msg::Pose relative;

    relative.position.x = 1;
    relative.position.y = -1;
    relative.position.z = -2;
    relative.orientation.w = 1;
    relative.orientation.x = 0;
    relative.orientation.y = 0;
    relative.orientation.z = -1;

    geometry_msgs::msg::Pose world;
    while(rclcpp::ok()) {
        try {
            geometry_msgs::msg::TransformStamped transform = buffer->lookupTransform("world", "test", tf2::TimePointZero);
            world = doTransform(relative, transform);
        } catch(tf2::LookupException& ex) {
            RCLCPP_INFO(log, "Lookup exception: %s", ex.what());
            rclcpp::spin_some(n);
        }
    }

    RCLCPP_INFO(log, "World Pose: %f, %f, %f with orientation %f %f %f %f",
        world.position.x,
        world.position.y,
        world.position.z,
        world.orientation.w,
        world.orientation.x,
        world.orientation.y,
        world.orientation.z    
    );

    rclcpp::shutdown();
    return 0;
}