#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/BufferedSubscriber.hpp"
#include "autonomy_test/TimedPublisher.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

TEST(BtTest, dummy_test) {
    //Subscription setup
    BufferedSubscriber<riptide_msgs2::msg::ControllerCommand> aggregator(BtTestEnvironment::getBtTestTool(), "controller/linear");

    //Node setup
    BT::NodeConfiguration nodeCfg;
    nodeCfg.input_ports["isOrientation"] = "0";
    nodeCfg.input_ports["mode"] = "3";
    nodeCfg.input_ports["x"] = "2";
    nodeCfg.input_ports["y"] = "4";
    nodeCfg.input_ports["z"] = "5";

    //test first-time publishes
    for(int i=0; i<5; i++) {
        auto node = BtTestEnvironment::getBtTestTool()->createLeafNodeFromConfig("PublishToController", nodeCfg);
        node->executeTick();
    }

    //expected value
    riptide_msgs2::msg::ControllerCommand expectedMsg;
    expectedMsg.mode = 3;

    geometry_msgs::msg::Vector3 expectedVec3;
    expectedVec3.x = 2;
    expectedVec3.y = 4;
    expectedVec3.z = 5;

    expectedMsg.setpoint_vect = expectedVec3;

    EXPECT_GT(numOccurrances<riptide_msgs2::msg::ControllerCommand>(aggregator.getMessages(), expectedMsg), 3);
}


TEST(BtTest, other_test) {
    //configure message to send
    nav_msgs::msg::Odometry odom;
    odom.pose.pose.position.x = 23.4;
    odom.pose.pose.position.y = 34.6;
    odom.pose.pose.position.z = -0.56;
    geometry_msgs::msg::Vector3 rpy;
    rpy.x = 34.5;
    rpy.y = 87.2;
    rpy.z = 0.04;
    odom.pose.pose.orientation = toQuat(rpy);

    //create timed publisher
    auto publisher = TimedPublisher<nav_msgs::msg::Odometry>(BtTestEnvironment::getBtTestTool(), "odometry/filtered", odom, 10, 250);

    //create bt node
    auto node = BtTestEnvironment::getBtTestTool()->createLeafNodeFromConfig("GetOdometry", BT::NodeConfiguration());
    
    //execute node until a timeout happens
    auto nodeStatus = BT::NodeStatus::RUNNING;
    auto startTime = BtTestEnvironment::getBtTestTool()->get_clock()->now();
    while(rclcpp::ok() && nodeStatus == BT::NodeStatus::RUNNING) {
        nodeStatus = node->executeTick();
    }

    ASSERT_EQ(nodeStatus, BT::NodeStatus::SUCCESS);
}


TEST(BtTest, node_execution_test) {
    BT::NodeConfiguration in;
    in.input_ports["a"] = "2";
    in.input_ports["b"] = "2";
    in.input_ports["operator"] = "+";

    auto node = BtTestEnvironment::getBtTestTool()->createLeafNodeFromConfig("Math", in);
    BT::NodeStatus result = node->executeTick();
    BT::Blackboard::Ptr out = node->config().blackboard;
    
    int defaultVal = 0;
    ASSERT_EQ(BT::NodeStatus::SUCCESS, result);
    ASSERT_EQ(getFromBlackboardWithDefault<int>(out, "out", defaultVal), 4);
}
