#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/DummyActionServer.hpp"

using namespace std::chrono_literals;
using ClawAction = riptide_msgs2::action::ChangeClawState;

class TestActuateClaw : public BtTest {
    protected:
    void SetUp() {
        BtTest::SetUp();
        serverNode = std::make_shared<rclcpp::Node>("dummy_server", "bt_testing");
        dummyServer = std::make_shared<DummyActionServer<ClawAction>>(serverNode, CLAW_SERVER_NAME);
    }

    void TearDown() {
        BtTest::TearDown();
        dummyServer.reset();
        serverNode.reset();
    }

    std::shared_ptr<DummyActionServer<ClawAction>> dummyServer;
    rclcpp::Node::SharedPtr serverNode; //node will house the dummy server
};


TEST_F(TestActuateClaw, test_ActuateClaw_open_success) {
    //spin up dummy action server
    dummyServer->configureNoFeedback(
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE,
        rclcpp_action::CancelResponse::REJECT,
        500ms,
        true,
        std::make_shared<ClawAction::Result>()
    );

    //create BT node
    BT::NodeConfiguration config;
    config.input_ports["claw_open"] = "1";
    auto node = toolNode->createLeafNodeFromConfig("ActuateClaw", config);

    //perform tests
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    while(rclcpp::ok() && (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING)) {
        rclcpp::spin_some(serverNode);
        rclcpp::spin_some(toolNode);
        status = node->executeTick();
    }

    //collect and analyze results
    bool receivedRequest = dummyServer->receivedRequest();

    ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(receivedRequest);

    //if a request was received, check it is correct
    ClawAction::Goal request = *dummyServer->getReceivedRequest();
    ASSERT_TRUE(request.clawopen);
}

TEST_F(TestActuateClaw, test_ActuateClaw_close_success) {
    //spin up dummy action server
    dummyServer->configureNoFeedback(
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE,
        rclcpp_action::CancelResponse::REJECT,
        500ms,
        true,
        std::make_shared<ClawAction::Result>()
    );

    //create BT node
    BT::NodeConfiguration config;
    config.input_ports["claw_open"] = "0";
    auto node = toolNode->createLeafNodeFromConfig("ActuateClaw", config);

    //perform tests
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    while(rclcpp::ok() && (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING)) {
        rclcpp::spin_some(serverNode);
        rclcpp::spin_some(toolNode);
        status = node->executeTick();
    }

    //collect and analyze results
    bool receivedRequest = dummyServer->receivedRequest();

    ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(receivedRequest);

    //if a request was received, check it is correct
    ClawAction::Goal request = *dummyServer->getReceivedRequest();
    ASSERT_FALSE(request.clawopen);
}

TEST_F(TestActuateClaw, test_ActuateClaw_fail_action_failed) {
    //spin up dummy action server
    dummyServer->configureNoFeedback(
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE,
        rclcpp_action::CancelResponse::REJECT,
        500ms,
        false,
        std::make_shared<ClawAction::Result>()
    );

    //create BT node
    BT::NodeConfiguration config;
    config.input_ports["claw_open"] = "0";
    auto node = toolNode->createLeafNodeFromConfig("ActuateClaw", config);

    //perform tests
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    while(rclcpp::ok() && (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING)) {
        rclcpp::spin_some(serverNode);
        rclcpp::spin_some(toolNode);
        status = node->executeTick();
    }

    //collect and analyze results
    bool receivedRequest = dummyServer->receivedRequest();

    ASSERT_EQ(status, BT::NodeStatus::FAILURE);
    ASSERT_TRUE(receivedRequest);
}

TEST_F(TestActuateClaw, test_ActuateClaw_fail_server_unavailable) {
    //spin up dummy action server
    dummyServer->configureNoFeedback(
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE,
        rclcpp_action::CancelResponse::REJECT,
        500ms,
        false,
        std::make_shared<ClawAction::Result>()
    );

    //create BT node
    BT::NodeConfiguration config;
    config.input_ports["claw_open"] = "0";
    auto node = toolNode->createLeafNodeFromConfig("ActuateClaw", config);

    //perform tests
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    while(rclcpp::ok() && (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING)) {
        rclcpp::spin_some(serverNode);
        rclcpp::spin_some(toolNode);
        status = node->executeTick();
    }

    //collect and analyze results
    bool receivedRequest = dummyServer->receivedRequest();

    ASSERT_EQ(status, BT::NodeStatus::FAILURE);
    ASSERT_TRUE(receivedRequest);
}
