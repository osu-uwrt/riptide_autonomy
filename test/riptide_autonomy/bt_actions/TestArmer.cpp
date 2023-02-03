#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/DummyActionServerWithFeedback.hpp"

using Armer = riptide_msgs2::action::ArmTorpedoDropper;

class ArmerTest : public BtTest {
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
        server = std::make_unique<DummyActionServerWithFeedback<Armer, Armer::Goal, Armer::Feedback, Armer::Result>>(toolNode, "arm_torpedo_dropper");
    }
    std::unique_ptr<DummyActionServerWithFeedback<Armer, Armer::Goal, Armer::Feedback, Armer::Result>> server;
};

/**
 * @brief Dropper node test.
 * 
 * @param dropper The status of the dropper
 * @param torpedo The status of the torpedo
 * @return BT::NodeStatus 
 */
BT::NodeStatus testDroppers(std::shared_ptr<BtTestTool> toolNode, bool torpedo, bool dropper) {
    //configure node
    BT::NodeConfiguration cfg;
    cfg.input_ports["ArmTorpedos"] = std::to_string(torpedo);
    cfg.input_ports["ArmDroppers"] = std::to_string(dropper);
    cfg.output_ports["IsArmed"];

    auto node = toolNode->createLeafNodeFromConfig("Armer", cfg);
    //run node
    auto startTime = toolNode->get_clock()->now();
    auto nodeResult = toolNode->tickUntilFinished(node);

    return nodeResult;
}

TEST_F(ArmerTest, test_arm_torpedo_success) {
    SetUpServer();
    server->configureExecution(
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE,
        rclcpp_action::CancelResponse::ACCEPT,
        1,
        std::make_shared<Armer::Result>(),
        std::make_shared<Armer::Feedback>()
    );

    auto result = testDroppers(toolNode, true, false);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
}
TEST_F(ArmerTest, test_arm_drpper_success) {
    SetUpServer();
    server->configureExecution(
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE,
        rclcpp_action::CancelResponse::ACCEPT,
        1,
        std::make_shared<Armer::Result>(),
        std::make_shared<Armer::Feedback>()
    );

    auto result = testDroppers(toolNode, false, true);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
}

TEST_F(ArmerTest, test_arm_twice_fail_1) {
    SetUpServer();
    server->configureExecution(
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE,
        rclcpp_action::CancelResponse::ACCEPT,
        1,
        std::make_shared<Armer::Result>(),
        std::make_shared<Armer::Feedback>()
    );

    auto result1 = testDroppers(toolNode, true, false);
    auto result2 = testDroppers(toolNode, true, false);

    ASSERT_EQ(result1, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(result2, BT::NodeStatus::FAILURE);
}

TEST_F(ArmerTest, test_arm_twice_fail_2) {
    SetUpServer();
    server->configureExecution(
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE,
        rclcpp_action::CancelResponse::ACCEPT,
        1,
        std::make_shared<Armer::Result>(),
        std::make_shared<Armer::Feedback>()
    );

    auto result1 = testDroppers(toolNode, true, false);
    auto result2 = testDroppers(toolNode, false, true);

    ASSERT_EQ(result1, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(result2, BT::NodeStatus::FAILURE);
}

TEST_F(ArmerTest, test_arm_twice_fail_3) {
    SetUpServer();
    server->configureExecution(
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE,
        rclcpp_action::CancelResponse::ACCEPT,
        1,
        std::make_shared<Armer::Result>(),
        std::make_shared<Armer::Feedback>()
    );

    auto result = testDroppers(toolNode, true, true);

    ASSERT_EQ(result, BT::NodeStatus::FAILURE);

}