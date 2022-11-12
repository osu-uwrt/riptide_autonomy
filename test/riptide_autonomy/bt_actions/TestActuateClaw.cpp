#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/DummyActionServer.hpp"

using ActuateClaw = riptide_msgs2::action::ChangeClawState;
using namespace std::chrono_literals;

class ActuateClawTest : public ::testing::Test {
    public:
    ActuateClawTest()
    : server(BtTestEnvironment::getBtTestTool(), CLAW_SERVER_NAME) {
        RCLCPP_INFO(log, "new server created.");
    }

    protected:
    DummyActionServer<ActuateClaw, ActuateClaw::Goal, ActuateClaw::Result> server;
};

/**
 * @brief Claw node test.
 * 
 * @param sendOpen True if the node client should send an open claw command, false for close claw.
 * @param secondsElapsed Will be populated with the number of seconds the node takes to complete
 * @return BT::NodeStatus 
 */
BT::NodeStatus testClaw(bool sendOpen, double& secondsElapsed) {
    //configure node
    BT::NodeConfiguration cfg;
    cfg.input_ports["claw_open"] = (sendOpen ? "1" : "0");
    auto node = BtTestEnvironment::getBtTestTool()->createLeafNodeFromConfig("ActuateClaw", cfg);

    //run node
    auto startTime = BtTestEnvironment::getBtTestTool()->get_clock()->now();
    auto nodeResult = BtTestEnvironment::getBtTestTool()->tickUntilFinished(node);
    secondsElapsed = (BtTestEnvironment::getBtTestTool()->get_clock()->now() - startTime).seconds();

    return nodeResult;
}

//important: this test is a regular TEST outside of the suite and before the suite is initialized, so the action server will not be running. 
//Node should fail after failing to find the server. 
TEST(ServerlessActuateClawTest, test_ActuateClaw_fail_not_available) {
    double seconds;
    auto result = testClaw(true, seconds);
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
    ASSERT_LT(seconds, 4);
}

TEST_F(ActuateClawTest, test_ActuateClaw_open_success) {
    server.configureExecution(
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE, 
        rclcpp_action::CancelResponse::ACCEPT, 
        3s, 
        std::make_shared<ActuateClaw::Result>()
    );
    
    double seconds;
    auto result = testClaw(true, seconds);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(server.getLatestGoal()->clawopen, true);
}

TEST_F(ActuateClawTest, test_ActuateClaw_close_success) {
    server.configureExecution(
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE, 
        rclcpp_action::CancelResponse::ACCEPT, 
        3s, 
        std::make_shared<ActuateClaw::Result>()
    );
    
    double seconds;
    auto result = testClaw(false, seconds);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(server.getLatestGoal()->clawopen, false);
}

TEST_F(ActuateClawTest, test_ActuateClaw_fail_timeout) {
    server.configureExecution(
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE, 
        rclcpp_action::CancelResponse::ACCEPT, 
        11s, 
        std::make_shared<ActuateClaw::Result>()
    );
    
    double seconds;
    auto result = testClaw(true, seconds);
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
}

TEST_F(ActuateClawTest, test_ActuateClaw_fail_rejected) {
    server.configureExecution(
        rclcpp_action::GoalResponse::REJECT, 
        rclcpp_action::CancelResponse::ACCEPT, 
        1s, 
        std::make_shared<ActuateClaw::Result>()
    );
    
    double seconds;
    auto result = testClaw(true, seconds);
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
    ASSERT_LT(seconds, 5);
}
