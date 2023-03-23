#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/DummyActionServer.hpp"

using namespace std::chrono_literals;
using DropperAction = riptide_msgs2::action::ActuateDroppers;

TEST_F(BtTest, test_ActuateDroppers_success_drop_1) {
    //spin up an extra ros node with a dummy action server
    auto serverNode = std::make_shared<rclcpp::Node>("server_node", "bt_testing");
    DummyActionServer<DropperAction> dummyServer(serverNode, DROPPER_SERVER_NAME);

    dummyServer.configureNoFeedback(
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE,
        rclcpp_action::CancelResponse::REJECT,
        500ms,
        true,
        std::make_shared<DropperAction::Result>()
    );

    //create bt node
    BT::NodeConfiguration config;
    config.input_ports["DropperID"] = "1";
    auto btNode = toolNode->createLeafNodeFromConfig("ActuateDroppers", config);

    BT::NodeStatus status = BT::NodeStatus::IDLE;
    while(rclcpp::ok() && (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING)) {
        rclcpp::spin_some(serverNode);
        rclcpp::spin_some(toolNode);
        status = btNode->executeTick();
    }

    ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(dummyServer.receivedRequest());

    //if request was received, check to make sure it was correct
    ASSERT_EQ(dummyServer.getReceivedRequest()->dropper_id, 1);
}

TEST_F(BtTest, test_ActuateDroppers_success_drop_2) {
    //spin up an extra ros node with a dummy action server
    auto serverNode = std::make_shared<rclcpp::Node>("server_node", "bt_testing");
    DummyActionServer<DropperAction> dummyServer(serverNode, DROPPER_SERVER_NAME);

    dummyServer.configureNoFeedback(
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE,
        rclcpp_action::CancelResponse::REJECT,
        500ms,
        true,
        std::make_shared<DropperAction::Result>()
    );

    //create bt node
    BT::NodeConfiguration config;
    config.input_ports["DropperID"] = "2";
    auto btNode = toolNode->createLeafNodeFromConfig("ActuateDroppers", config);

    BT::NodeStatus status = BT::NodeStatus::IDLE;
    while(rclcpp::ok() && (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING)) {
        rclcpp::spin_some(serverNode);
        rclcpp::spin_some(toolNode);
        status = btNode->executeTick();
    }

    ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(dummyServer.receivedRequest());

    //if request was received, check to make sure it was correct
    ASSERT_EQ(dummyServer.getReceivedRequest()->dropper_id, 2);
}

TEST_F(BtTest, test_ActuateDroppers_fail_action_failed) {
    //spin up an extra ros node with a dummy action server
    auto serverNode = std::make_shared<rclcpp::Node>("server_node", "bt_testing");
    DummyActionServer<DropperAction> dummyServer(serverNode, DROPPER_SERVER_NAME);

    dummyServer.configureNoFeedback(
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE,
        rclcpp_action::CancelResponse::REJECT,
        500ms,
        false,
        std::make_shared<DropperAction::Result>()
    );

    //create bt node
    BT::NodeConfiguration config;
    config.input_ports["DropperID"] = "1";
    auto btNode = toolNode->createLeafNodeFromConfig("ActuateDroppers", config);

    BT::NodeStatus status = BT::NodeStatus::IDLE;
    while(rclcpp::ok() && (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING)) {
        rclcpp::spin_some(serverNode);
        rclcpp::spin_some(toolNode);
        status = btNode->executeTick();
    }

    ASSERT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(BtTest, test_ActuateDroppers_fail_unavailable) {
    //create bt node
    BT::NodeConfiguration config;
    config.input_ports["DropperID"] = "1";
    auto btNode = toolNode->createLeafNodeFromConfig("ActuateDroppers", config);

    BT::NodeStatus status = BT::NodeStatus::IDLE;
    while(rclcpp::ok() && (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING)) {
        rclcpp::spin_some(toolNode);
        status = btNode->executeTick();
    }

    ASSERT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(BtTest, test_ActuateDroppers_fail_rejected) {
    //spin up an extra ros node with a dummy action server
    auto serverNode = std::make_shared<rclcpp::Node>("server_node", "bt_testing");
    DummyActionServer<DropperAction> dummyServer(serverNode, DROPPER_SERVER_NAME);

    dummyServer.configureNoFeedback(
        rclcpp_action::GoalResponse::REJECT,
        rclcpp_action::CancelResponse::REJECT,
        5s,
        true,
        std::make_shared<DropperAction::Result>()
    );

    //create bt node
    BT::NodeConfiguration config;
    config.input_ports["DropperID"] = "1";
    auto btNode = toolNode->createLeafNodeFromConfig("ActuateDroppers", config);

    BT::NodeStatus status = BT::NodeStatus::IDLE;
    while(rclcpp::ok() && (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING)) {
        rclcpp::spin_some(serverNode);
        rclcpp::spin_some(toolNode);
        status = btNode->executeTick();
    }

    ASSERT_EQ(status, BT::NodeStatus::FAILURE);
    ASSERT_LT(testElapsed().seconds(), 2); //assert that the server didnt just time out or was unavailable
}

TEST_F(BtTest, test_ActuateDroppers_fail_timed_out) {
    //spin up an extra ros node with a dummy action server
    auto serverNode = std::make_shared<rclcpp::Node>("server_node", "bt_testing");
    DummyActionServer<DropperAction> dummyServer(serverNode, DROPPER_SERVER_NAME);

    dummyServer.configureNoFeedback(
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE,
        rclcpp_action::CancelResponse::REJECT,
        15s,
        true,
        std::make_shared<DropperAction::Result>()
    );

    //create bt node
    BT::NodeConfiguration config;
    config.input_ports["DropperID"] = "1";
    auto btNode = toolNode->createLeafNodeFromConfig("ActuateDroppers", config);

    BT::NodeStatus status = BT::NodeStatus::IDLE;
    while(rclcpp::ok() && (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING)) {
        rclcpp::spin_some(serverNode);
        rclcpp::spin_some(toolNode);
        status = btNode->executeTick();
    }

    //kill the action or else it will segfault
    dummyServer.forceStop();
    rclcpp::sleep_for(50ms); //give action server time to die

    ASSERT_EQ(status, BT::NodeStatus::FAILURE);
    ASSERT_GT(testElapsed().seconds(), 8); //ensure that the node actually timed out
}
