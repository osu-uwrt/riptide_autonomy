#include "riptide_autonomy/bt_actions/PublishEKFPose.btaction.hpp"
#include <behaviortree_cpp_v3/controls/sequence_node.h>
#include <gtest/gtest.h>
#include <thread>

class PoseResetTest : public ::testing::Test {
protected:
    using SetPose = robot_localization::srv::SetPose;
    using Odometry = nav_msgs::msg::Odometry;

    void SetUp() override {
        rclcpp::init(0, nullptr);
        node = std::make_shared<rclcpp::Node>("pose_reset_test", "/pose_reset_test");
        // Defer the reply so the test controls service and odometry ordering.
        service = node->create_service<SetPose>("set_pose",
            [this](std::shared_ptr<rmw_request_id_t> header, SetPose::Request::SharedPtr value) {
                requestHeader = header;
                request = value;
                ++requests;
            });
        publisher = node->create_publisher<Odometry>("odometry/filtered", 10);
        config.blackboard = BT::Blackboard::create();
        config.input_ports = {{"setX", "true"}, {"setY", "true"}, {"setZ", "false"},
            {"setOrientation", "true"}, {"x", "2"}, {"y", "3"},
            {"roll", "0"}, {"pitch", "0"}, {"yaw", "0.5"}, {"timeout_secs", "2"}};
    }

    void TearDown() override {
        action.reset();
        publisher.reset();
        service.reset();
        node.reset();
        rclcpp::shutdown();
    }

    void pump(std::chrono::milliseconds duration = 40ms) {
        const auto deadline = std::chrono::steady_clock::now() + duration;
        do {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(1ms);
        } while (std::chrono::steady_clock::now() < deadline);
    }

    void start() {
        action = std::make_unique<PublishEKFPose>("reset", config);
        action->init(node);
        const auto deadline = std::chrono::steady_clock::now() + 2s;
        while (publisher->get_subscription_count() == 0 && std::chrono::steady_clock::now() < deadline)
            pump(10ms);
        ASSERT_GT(publisher->get_subscription_count(), 0u);
        ASSERT_EQ(action->executeTick(), BT::NodeStatus::RUNNING);
    }

    void publish(Odometry odom, bool fresh = true) {
        if (fresh) odom.header.stamp = node->now();
        publisher->publish(odom);
        pump();
    }

    Odometry drifted() {
        Odometry odom;
        odom.header.frame_id = "odom";
        odom.pose.pose.position.x = 8;
        odom.pose.pose.position.y = 9;
        odom.pose.pose.position.z = -1.2;
        odom.pose.pose.orientation.w = 1;
        return odom;
    }

    Odometry corrected() {
        Odometry odom;
        odom.header.frame_id = "odom";
        odom.pose = request->pose.pose;
        return odom;
    }

    void sendRequest() {
        publish(drifted());
        ASSERT_EQ(action->executeTick(), BT::NodeStatus::RUNNING);
        const auto deadline = std::chrono::steady_clock::now() + 1s;
        while (!request && std::chrono::steady_clock::now() < deadline) pump(10ms);
        ASSERT_NE(request, nullptr);
    }

    void reply() {
        SetPose::Response response;
        service->send_response(*requestHeader, response);
        pump();
        ASSERT_EQ(action->executeTick(), BT::NodeStatus::RUNNING);
    }

    rclcpp::Node::SharedPtr node;
    rclcpp::Service<SetPose>::SharedPtr service;
    rclcpp::Publisher<Odometry>::SharedPtr publisher;
    std::unique_ptr<PublishEKFPose> action;
    BT::NodeConfiguration config;
    std::shared_ptr<rmw_request_id_t> requestHeader;
    SetPose::Request::SharedPtr request;
    int requests = 0;
};

TEST_F(PoseResetTest, WaitsForReplyAndFreshCorrectedOdometry) {
    start();
    sendRequest();
    ASSERT_NE(request, nullptr);
    EXPECT_DOUBLE_EQ(request->pose.pose.pose.position.z, -1.2);
    // Even matching odometry cannot finish an unacknowledged reset.
    publish(corrected());
    EXPECT_EQ(action->executeTick(), BT::NodeStatus::RUNNING);
    reply();
    EXPECT_EQ(action->executeTick(), BT::NodeStatus::RUNNING);
    // This is the flip race: a new message still contains the drifted pose.
    publish(drifted());
    EXPECT_EQ(action->executeTick(), BT::NodeStatus::RUNNING);
    auto old = corrected();
    old.header.stamp = request->pose.header.stamp;
    publish(old, false);
    EXPECT_EQ(action->executeTick(), BT::NodeStatus::RUNNING);
    auto wrongFrame = corrected();
    wrongFrame.header.frame_id = "map";
    publish(wrongFrame);
    EXPECT_EQ(action->executeTick(), BT::NodeStatus::RUNNING);
    auto wrongYaw = corrected();
    wrongYaw.pose.pose.orientation = drifted().pose.pose.orientation;
    publish(wrongYaw);
    EXPECT_EQ(action->executeTick(), BT::NodeStatus::RUNNING);
    auto valid = corrected();
    valid.pose.pose.position.z = -1.4; // Z was not selected for correction.
    valid.pose.pose.orientation.z *= -1;
    valid.pose.pose.orientation.w *= -1; // Quaternion sign must not matter.
    publish(valid);
    EXPECT_EQ(action->executeTick(), BT::NodeStatus::SUCCESS);
    EXPECT_EQ(requests, 1);
}

TEST_F(PoseResetTest, TimesOutWithoutOdometryEvenWhenRosTimeIsPaused) {
    config.input_ports["timeout_secs"] = "0.15";
    node->set_parameter(rclcpp::Parameter("use_sim_time", true));
    start();
    pump(180ms);
    EXPECT_EQ(action->executeTick(), BT::NodeStatus::FAILURE);
    EXPECT_EQ(requests, 0);
}

TEST_F(PoseResetTest, TimesOutWithoutServiceReply) {
    config.input_ports["timeout_secs"] = "0.3";
    start();
    sendRequest();
    ASSERT_NE(request, nullptr);
    publish(corrected());
    pump(320ms);
    EXPECT_EQ(action->executeTick(), BT::NodeStatus::FAILURE);
    EXPECT_EQ(requests, 1);
}

TEST_F(PoseResetTest, TimesOutWhenCorrectionNeverAppears) {
    config.input_ports["timeout_secs"] = "0.3";
    start();
    sendRequest();
    ASSERT_NE(request, nullptr);
    reply();
    publish(drifted());
    pump(320ms);
    EXPECT_EQ(action->executeTick(), BT::NodeStatus::FAILURE);
    EXPECT_EQ(requests, 1);
}

TEST_F(PoseResetTest, HaltAndRestartRequireANewRequestAndOdometry) {
    start();
    sendRequest();
    ASSERT_NE(request, nullptr);
    BT::SequenceNode parent("sequence");
    parent.addChild(action.get());
    parent.haltChild(0); // The parent also returns the child to IDLE.
    // A late response to the halted request must not complete the next reset.
    SetPose::Response response;
    service->send_response(*requestHeader, response);
    pump();
    request.reset();
    EXPECT_EQ(action->executeTick(), BT::NodeStatus::RUNNING);
    EXPECT_EQ(action->executeTick(), BT::NodeStatus::RUNNING);
    EXPECT_EQ(requests, 1);
    sendRequest();
    ASSERT_NE(request, nullptr);
    reply();
    publish(corrected());
    EXPECT_EQ(action->executeTick(), BT::NodeStatus::SUCCESS);
    EXPECT_EQ(requests, 2);
}
