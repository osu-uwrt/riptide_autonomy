#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/DummyActionServer.hpp"

using namespace std::chrono_literals;
using FireTorpedo = riptide_msgs2::action::ActuateTorpedos;

TEST_F(BtTest, test_FireTorpedos_success_fire_1) {
    //spin up a new node with a dummy server
    auto serverNode = std::make_shared<rclcpp::Node>("server_node", "bt_testing");
    DummyActionServer<FireTorpedo> dummyServer(serverNode, TORPEDO_SERVER_NAME);
    dummyServer.configureNoFeedback(
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE,
        rclcpp_action::CancelResponse::REJECT,
        500ms,
        true,
        std::make_shared<FireTorpedo::Result>()
    );

    //create BT node to test
    BT::NodeConfiguration config;
    config.input_ports["TorpedoID"] = "1";
    auto node = toolNode->createLeafNodeFromConfig("FireTorpedos", config);

    //tick until node is finished
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    while(rclcpp::ok() && (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING)) {
        rclcpp::spin_some(serverNode);
        rclcpp::spin_some(toolNode);
        status = node->executeTick();
    }

    //analyze results
    ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(dummyServer.receivedRequest());

    //if request was received, verify if it is correct
    ASSERT_EQ(dummyServer.getReceivedRequest()->torpedo_id, 1);
}

TEST_F(BtTest, test_FireTorpedos_success_fire_2) {
    //spin up a new node with a dummy server
    auto serverNode = std::make_shared<rclcpp::Node>("server_node", "bt_testing");
    DummyActionServer<FireTorpedo> dummyServer(serverNode, TORPEDO_SERVER_NAME);
    dummyServer.configureNoFeedback(
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE,
        rclcpp_action::CancelResponse::REJECT,
        500ms,
        true,
        std::make_shared<FireTorpedo::Result>()
    );

    //create BT node to test
    BT::NodeConfiguration config;
    config.input_ports["TorpedoID"] = "2";
    auto node = toolNode->createLeafNodeFromConfig("FireTorpedos", config);

    //tick until node is finished
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    while(rclcpp::ok() && (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING)) {
        rclcpp::spin_some(serverNode);
        rclcpp::spin_some(toolNode);
        status = node->executeTick();
    }

    //analyze results
    ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(dummyServer.receivedRequest());

    //if request was received, verify if it is correct
    ASSERT_EQ(dummyServer.getReceivedRequest()->torpedo_id, 2);
}

TEST_F(BtTest, test_FireTorpedos_fail_action_failed) {
    //spin up a new node with a dummy server
    auto serverNode = std::make_shared<rclcpp::Node>("server_node", "bt_testing");
    DummyActionServer<FireTorpedo> dummyServer(serverNode, TORPEDO_SERVER_NAME);
    dummyServer.configureNoFeedback(
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE,
        rclcpp_action::CancelResponse::REJECT,
        500ms,
        false,
        std::make_shared<FireTorpedo::Result>()
    );

    //create BT node to test
    BT::NodeConfiguration config;
    config.input_ports["TorpedoID"] = "1";
    auto node = toolNode->createLeafNodeFromConfig("FireTorpedos", config);

    //tick until node is finished
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    while(rclcpp::ok() && (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING)) {
        rclcpp::spin_some(serverNode);
        rclcpp::spin_some(toolNode);
        status = node->executeTick();
    }

    //analyze results
    ASSERT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(BtTest, test_FireTorpedos_fail_rejected) {
    //spin up a new node with a dummy server
    auto serverNode = std::make_shared<rclcpp::Node>("server_node", "bt_testing");
    DummyActionServer<FireTorpedo> dummyServer(serverNode, TORPEDO_SERVER_NAME);
    dummyServer.configureNoFeedback(
        rclcpp_action::GoalResponse::REJECT,
        rclcpp_action::CancelResponse::REJECT,
        5s,
        true,
        std::make_shared<FireTorpedo::Result>()
    );

    //create BT node to test
    BT::NodeConfiguration config;
    config.input_ports["TorpedoID"] = "1";
    auto node = toolNode->createLeafNodeFromConfig("FireTorpedos", config);

    //tick until node is finished
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    while(rclcpp::ok() && (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING)) {
        rclcpp::spin_some(serverNode);
        rclcpp::spin_some(toolNode);
        status = node->executeTick();
    }

    //analyze results
    ASSERT_EQ(status, BT::NodeStatus::FAILURE);
    ASSERT_LT(testElapsed().seconds(), 2);
}

TEST_F(BtTest, test_FireTorpedos_fail_unavailable) {
    //create BT node to test
    BT::NodeConfiguration config;
    config.input_ports["TorpedoID"] = "1";
    auto node = toolNode->createLeafNodeFromConfig("FireTorpedos", config);

    //tick until node is finished
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    while(rclcpp::ok() && (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING)) {
        rclcpp::spin_some(toolNode);
        status = node->executeTick();
    }

    //analyze results
    ASSERT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(BtTest, test_FireTorpedos_fail_timed_out) {
    //spin up a new node with a dummy server
    auto serverNode = std::make_shared<rclcpp::Node>("server_node", "bt_testing");
    DummyActionServer<FireTorpedo> dummyServer(serverNode, TORPEDO_SERVER_NAME);
    dummyServer.configureNoFeedback(
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE,
        rclcpp_action::CancelResponse::REJECT,
        15s,
        true,
        std::make_shared<FireTorpedo::Result>()
    );

    //create BT node to test
    BT::NodeConfiguration config;
    config.input_ports["TorpedoID"] = "1";
    auto node = toolNode->createLeafNodeFromConfig("FireTorpedos", config);

    //tick until node is finished
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    while(rclcpp::ok() && (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING)) {
        rclcpp::spin_some(serverNode);
        rclcpp::spin_some(toolNode);
        status = node->executeTick();
    }

    //kill the action or else it will segfault
    dummyServer.forceStop();
    rclcpp::sleep_for(50ms); //give action server time to die

    //analyze results
    ASSERT_EQ(status, BT::NodeStatus::FAILURE);
    ASSERT_GT(testElapsed().seconds(), 8);
}
