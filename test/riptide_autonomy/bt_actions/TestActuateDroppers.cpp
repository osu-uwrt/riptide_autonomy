#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/DummyActionServer.hpp"

using ActuateDroppers = riptide_msgs2::action::ActuateDroppers;

class ActuateDroppersTest : public BtTest {
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
        server = std::make_unique<DummyActionServer<ActuateDroppers, ActuateDroppers::Goal, ActuateDroppers::Result>>(toolNode, DROPPER_SERVER_NAME);
    }
    std::unique_ptr<DummyActionServer<ActuateDroppers, ActuateDroppers::Goal, ActuateDroppers::Result>> server;
};

/**
 * @brief Dropper node test.
 * 
 * @param id The ID of the dropper to "drop"
 * @param secondsElapsed Will be populated with the number of seconds the node takes to complete
 * @return BT::NodeStatus 
 */
BT::NodeStatus testDroppers(std::shared_ptr<BtTestTool> toolNode, int id, double& secondsElapsed) {
    //configure node
    BT::NodeConfiguration cfg;
    cfg.input_ports["DropperID"] = std::to_string(id);
    auto node = toolNode->createLeafNodeFromConfig("ActuateDroppers", cfg);

    //run node
    auto startTime = toolNode->get_clock()->now();
    auto nodeResult = toolNode->tickUntilFinished(node);
    secondsElapsed = (toolNode->get_clock()->now() - startTime).seconds();

    return nodeResult;
}

TEST_F(ActuateDroppersTest, test_ActuateDroppers_success_1) {
    SetUpServer();
    server->configureExecution(
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE,
        rclcpp_action::CancelResponse::ACCEPT,
        1,
        std::make_shared<ActuateDroppers::Result>()
    );

    double seconds;
    auto result = testDroppers(toolNode, 1, seconds);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(server->getLatestGoal()->dropper_id, 1);
}

TEST_F(ActuateDroppersTest, test_ActuateDroppers_success_2) {
    SetUpServer();
    server->configureExecution(
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE,
        rclcpp_action::CancelResponse::ACCEPT,
        1,
        std::make_shared<ActuateDroppers::Result>()
    );

    double seconds;
    auto result = testDroppers(toolNode, 2, seconds);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(server->getLatestGoal()->dropper_id, 2);
}

TEST_F(ActuateDroppersTest, test_ActuateDroppers_fail_timeout) {
    SetUpServer();
    server->configureExecution(
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE,
        rclcpp_action::CancelResponse::ACCEPT,
        11,
        std::make_shared<ActuateDroppers::Result>()
    );

    double seconds;
    auto result = testDroppers(toolNode, 0, seconds);
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
    ASSERT_NEAR(seconds, 10, 0.2);

    rclcpp::sleep_for(2s);
}

TEST_F(ActuateDroppersTest, test_ActuateDroppers_fail_rejected) {
    SetUpServer();
    server->configureExecution(
        rclcpp_action::GoalResponse::REJECT,
        rclcpp_action::CancelResponse::ACCEPT,
        3,
        std::make_shared<ActuateDroppers::Result>()
    );

    double seconds;
    auto result = testDroppers(toolNode, 0, seconds);
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
    ASSERT_LT(seconds, 3);
}

TEST_F(ActuateDroppersTest, test_ActuateDroppers_fail_not_available) {
    double seconds;
    auto result = testDroppers(toolNode, true, seconds);
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
    ASSERT_NEAR(seconds, 3, 0.2);
}
