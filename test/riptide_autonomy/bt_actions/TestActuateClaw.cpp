#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/DummyActionServer.hpp"

using ActuateClaw = riptide_msgs2::action::ChangeClawState;
using namespace std::chrono_literals;

class ActuateClawTest : public BtTest {
    public:
    void SetUp() override {
        BtTest::SetUp();
    }

    void TearDown() override {
        server.reset();
        BtTest::TearDown();
    }

    protected:
    void SetUpServer() {
        server = std::make_unique<DummyActionServer<ActuateClaw, ActuateClaw::Goal, ActuateClaw::Result>>(toolNode, CLAW_SERVER_NAME);
    }
    std::unique_ptr<DummyActionServer<ActuateClaw, ActuateClaw::Goal, ActuateClaw::Result>> server;
};

/**
 * @brief Claw node test.
 * 
 * @param sendOpen True if the node client should send an open claw command, false for close claw.
 * @param secondsElapsed Will be populated with the number of seconds the node takes to complete
 * @return BT::NodeStatus 
 */
BT::NodeStatus testClaw(std::shared_ptr<BtTestTool> toolNode, bool sendOpen, double& secondsElapsed) {
    //configure node
    BT::NodeConfiguration cfg;
    cfg.input_ports["claw_open"] = (sendOpen ? "1" : "0");
    auto node = toolNode->createLeafNodeFromConfig("ActuateClaw", cfg);

    //run node
    auto startTime = toolNode->get_clock()->now();
    auto nodeResult = toolNode->tickUntilFinished(node);
    secondsElapsed = (toolNode->get_clock()->now() - startTime).seconds();

    return nodeResult;
}

TEST_F(ActuateClawTest, test_ActuateClaw_open_success) {
    SetUpServer();
    server->configureExecution(
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE, 
        rclcpp_action::CancelResponse::ACCEPT, 
        3, 
        std::make_shared<ActuateClaw::Result>()
    );
    
    double seconds;
    auto result = testClaw(toolNode, true, seconds);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(server->getLatestGoal()->clawopen, true);
}

TEST_F(ActuateClawTest, test_ActuateClaw_close_success) {
    SetUpServer();
    server->configureExecution(
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE, 
        rclcpp_action::CancelResponse::ACCEPT, 
        3, 
        std::make_shared<ActuateClaw::Result>()
    );
    
    double seconds;
    auto result = testClaw(toolNode, false, seconds);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(server->getLatestGoal()->clawopen, false);
}

TEST_F(ActuateClawTest, test_ActuateClaw_fail_timeout) {
    SetUpServer();
    server->configureExecution(
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE, 
        rclcpp_action::CancelResponse::ACCEPT, 
        11, 
        std::make_shared<ActuateClaw::Result>()
    );
    
    double seconds;
    auto result = testClaw(toolNode, true, seconds);
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
    ASSERT_NEAR(seconds, 10, 0.2);

    rclcpp::sleep_for(2s); //allow action to finish to avoid it getting confused
}

TEST_F(ActuateClawTest, test_ActuateClaw_fail_rejected) {
    SetUpServer();
    server->configureExecution(
        rclcpp_action::GoalResponse::REJECT, 
        rclcpp_action::CancelResponse::ACCEPT, 
        3,
        std::make_shared<ActuateClaw::Result>()
    );
    
    double seconds;
    auto result = testClaw(toolNode, true, seconds);
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
    ASSERT_LT(seconds, 3);
}

TEST_F(ActuateClawTest, test_ActuateClaw_fail_not_available) {
    double seconds;
    auto result = testClaw(toolNode, true, seconds);
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
    ASSERT_NEAR(seconds, 3, 0.2);
}
