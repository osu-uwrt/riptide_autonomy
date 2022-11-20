#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/DummyActionServer.hpp"

using ActuateTorpedos = riptide_msgs2::action::ActuateTorpedos;

class FireTorpedoTest : public BtTest {
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
        server = std::make_unique<DummyActionServer<ActuateTorpedos, ActuateTorpedos::Goal, ActuateTorpedos::Result>>(toolNode, TORPEDO_SERVER_NAME);
    }
    std::unique_ptr<DummyActionServer<ActuateTorpedos, ActuateTorpedos::Goal, ActuateTorpedos::Result>> server;
};


/**
 * @brief Torpedo node test.
 * 
 * @param id The ID of the torpedo to "fire"
 * @param secondsElapsed Will be populated with the number of seconds the node takes to complete
 * @return BT::NodeStatus 
 */
BT::NodeStatus testTorpedos(std::shared_ptr<BtTestTool> toolNode, int id, double& secondsElapsed) {
    //configure node
    BT::NodeConfiguration cfg;
    cfg.input_ports["TorpedoID"] = std::to_string(id);
    auto node = toolNode->createLeafNodeFromConfig("FireTorpedos", cfg);

    //run node
    auto startTime = toolNode->get_clock()->now();
    auto nodeResult = toolNode->tickUntilFinished(node);
    secondsElapsed = (toolNode->get_clock()->now() - startTime).seconds();

    return nodeResult;
}


TEST_F(FireTorpedoTest, test_ActuateTorpedos_success_1) {
    SetUpServer();
    server->configureExecution(
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE,
        rclcpp_action::CancelResponse::ACCEPT,
        1,
        std::make_shared<ActuateTorpedos::Result>()
    );

    double seconds;
    auto result = testTorpedos(toolNode, 1, seconds);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(server->getLatestGoal()->torpedo_id, 1);
}

TEST_F(FireTorpedoTest, test_ActuateTorpedos_success_2) {
    SetUpServer();
    server->configureExecution(
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE,
        rclcpp_action::CancelResponse::ACCEPT,
        1,
        std::make_shared<ActuateTorpedos::Result>()
    );

    double seconds;
    auto result = testTorpedos(toolNode, 2, seconds);
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(server->getLatestGoal()->torpedo_id, 2);
}

TEST_F(FireTorpedoTest, test_ActuateTorpedos_fail_timeout) {
    SetUpServer();
    server->configureExecution(
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE,
        rclcpp_action::CancelResponse::ACCEPT,
        11,
        std::make_shared<ActuateTorpedos::Result>()
    );

    double seconds;
    auto result = testTorpedos(toolNode, 0, seconds);
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
    ASSERT_NEAR(seconds, 10, 0.2);

    rclcpp::sleep_for(2s);
}

TEST_F(FireTorpedoTest, test_ActuateTorpedos_fail_rejected) {
    SetUpServer();
    server->configureExecution(
        rclcpp_action::GoalResponse::REJECT,
        rclcpp_action::CancelResponse::ACCEPT,
        3,
        std::make_shared<ActuateTorpedos::Result>()
    );

    double seconds;
    auto result = testTorpedos(toolNode, 0, seconds);
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
    ASSERT_LT(seconds, 3);
}

TEST_F(FireTorpedoTest, test_ActuateTorpedos_fail_not_available) {
    double seconds;
    auto result = testTorpedos(toolNode, true, seconds);
    ASSERT_EQ(result, BT::NodeStatus::FAILURE);
    ASSERT_NEAR(seconds, 3, 0.2);
}
