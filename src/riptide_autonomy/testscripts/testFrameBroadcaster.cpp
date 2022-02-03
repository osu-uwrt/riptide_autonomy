#include "autonomy.h"
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;

class FrameBroadcaster : public rclcpp::Node {
    public:
    FrameBroadcaster()
     : Node("FrameBroadcaster") {
        this->timer = this->create_wall_timer(1s, std::bind(&FrameBroadcaster::timerCB, this));
        this->transBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    private:
    rclcpp::TimerBase::SharedPtr timer;
    std::unique_ptr<tf2_ros::TransformBroadcaster> transBroadcaster;

    void timerCB() {
        geometry_msgs::msg::TransformStamped frame;
        frame.header.stamp = this->get_clock()->now();
        frame.header.frame_id = "world";
        frame.child_frame_id = "test";
        frame.transform.translation.x = 2;
        frame.transform.translation.y = 4;
        frame.transform.translation.z = 5;
        frame.transform.rotation.w = 1;
        frame.transform.rotation.x = 0;
        frame.transform.rotation.y = 0;
        frame.transform.rotation.z = -1;

        transBroadcaster->sendTransform(frame);
        RCLCPP_INFO(log, "Sent Transform.");
    }
};


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrameBroadcaster>());
    rclcpp::shutdown();
}