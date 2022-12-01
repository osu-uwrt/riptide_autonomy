#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/DummyActionServer.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
    using DropperMsg = riptide_msgs2::action::ActuateDroppers;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("some_node");
    DummyActionServer<DropperMsg, DropperMsg::Goal, DropperMsg::Result> dummyServer(node, "some_server");
    dummyServer.configureExecution(rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE, rclcpp_action::CancelResponse::ACCEPT, 10, std::make_shared<DropperMsg::Result>());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
