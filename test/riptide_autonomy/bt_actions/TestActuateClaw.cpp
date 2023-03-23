#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/DummyActionServer.hpp"

using namespace std::chrono_literals;
using ClawAction = riptide_msgs2::action::ChangeClawState;


TEST_F(BtTest, test_ActuateClaw_open_success) {
    //spin up an extra ros node with a dummy action server
    auto serverNode = std::make_shared<rclcpp::Node>("dummy_server", "bt_testing");
    DummyActionServer<ClawAction> dummyServer(serverNode, CLAW_SERVER_NAME);

    dummyServer.configureNoFeedback(
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

    ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(dummyServer.receivedRequest());

    //if a request was received, check it is correct
    ClawAction::Goal request = *dummyServer.getReceivedRequest();
    ASSERT_TRUE(request.clawopen);
}

TEST_F(BtTest, test_ActuateClaw_close_success) {
    //spin up an extra ros node with a dummy action server
    auto serverNode = std::make_shared<rclcpp::Node>("dummy_server", "bt_testing");
    DummyActionServer<ClawAction> dummyServer(serverNode, CLAW_SERVER_NAME);

    dummyServer.configureNoFeedback(
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
    bool receivedRequest = dummyServer.receivedRequest();

    ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(receivedRequest);

    //if a request was received, check it is correct
    ClawAction::Goal request = *dummyServer.getReceivedRequest();
    ASSERT_FALSE(request.clawopen);
}

TEST_F(BtTest, test_ActuateClaw_fail_action_failed_opening) {
    //spin up an extra ros node with a dummy action server
    auto serverNode = std::make_shared<rclcpp::Node>("dummy_server", "bt_testing");
    DummyActionServer<ClawAction> dummyServer(serverNode, CLAW_SERVER_NAME);

    dummyServer.configureNoFeedback(
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE,
        rclcpp_action::CancelResponse::REJECT,
        500ms,
        false,
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
    bool receivedRequest = dummyServer.receivedRequest();

    ASSERT_EQ(status, BT::NodeStatus::FAILURE);
    ASSERT_TRUE(receivedRequest);
}

TEST_F(BtTest, test_ActuateClaw_fail_action_failed_closing) {
    //spin up an extra ros node with a dummy action server
    auto serverNode = std::make_shared<rclcpp::Node>("dummy_server", "bt_testing");
    DummyActionServer<ClawAction> dummyServer(serverNode, CLAW_SERVER_NAME);

    dummyServer.configureNoFeedback(
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
    bool receivedRequest = dummyServer.receivedRequest();

    ASSERT_EQ(status, BT::NodeStatus::FAILURE);
    ASSERT_TRUE(receivedRequest);
}

TEST_F(BtTest, test_ActuateClaw_fail_server_unavailable) {
    //create BT node
    BT::NodeConfiguration config;
    config.input_ports["claw_open"] = "0";
    auto node = toolNode->createLeafNodeFromConfig("ActuateClaw", config);

    //perform tests
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    while(rclcpp::ok() && (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING)) {
        rclcpp::spin_some(toolNode);
        status = node->executeTick();
    }

    ASSERT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(BtTest, test_ActuateClaw_fail_rejected) {
    //spin up an extra ros node with a dummy action server
    auto serverNode = std::make_shared<rclcpp::Node>("dummy_server", "bt_testing");
    DummyActionServer<ClawAction> dummyServer(serverNode, CLAW_SERVER_NAME);

    dummyServer.configureNoFeedback(
        rclcpp_action::GoalResponse::REJECT,
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
    bool receivedRequest = dummyServer.receivedRequest();

    ASSERT_EQ(status, BT::NodeStatus::FAILURE);
    ASSERT_TRUE(receivedRequest);
    ASSERT_TRUE(testElapsed() < 2s); //checks if the node failed because of rejection or timeout
}

TEST_F(BtTest, test_ActuateClaw_fail_timed_out) {
    //spin up an extra ros node with a dummy action server
    auto serverNode = std::make_shared<rclcpp::Node>("dummy_server", "bt_testing");
    DummyActionServer<ClawAction> dummyServer(serverNode, CLAW_SERVER_NAME);

    dummyServer.configureNoFeedback(
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE,
        rclcpp_action::CancelResponse::REJECT,
        30s,
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

    dummyServer.forceStop();
    rclcpp::sleep_for(50ms); //give action server time to die

    //collect and analyze results
    bool receivedRequest = dummyServer.receivedRequest();

    ASSERT_EQ(status, BT::NodeStatus::FAILURE);
    ASSERT_TRUE(receivedRequest);
    ASSERT_TRUE(testElapsed() > 5s); //tests if the node actually timed out or if something else happened
}
