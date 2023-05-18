#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/DummyService.hpp"

using namespace std::chrono_literals;
using SetPose = robot_localization::srv::SetPose;
using TestSetPose = ServiceTest<SetPose>;

std::chrono::duration<double> TESTRESETODOM_TIMEOUT = 7s;
const double TESTRESETODOM_EPSILON = 0.01;

BT::NodeStatus testResetOdom(
    std::shared_ptr<BtTestTool> toolNode,
    double x,
    double y,
    double z,
    double roll,
    double pitch,
    double yaw)
{
    //set up BT node
    BT::NodeConfiguration cfg;
    cfg.input_ports["x"] = std::to_string(x);
    cfg.input_ports["y"] = std::to_string(y);
    cfg.input_ports["z"] = std::to_string(z);
    cfg.input_ports["or"] = std::to_string(roll);
    cfg.input_ports["op"] = std::to_string(pitch);
    cfg.input_ports["oy"] = std::to_string(yaw);

    auto node = toolNode->createLeafNodeFromConfig("ResetOdom", cfg);

    //run bt node until complete
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    rclcpp::Time startTime = toolNode->get_clock()->now();
    while(status != BT::NodeStatus::SUCCESS && status != BT::NodeStatus::FAILURE && toolNode->get_clock()->now() - startTime < TESTRESETODOM_TIMEOUT) {
        status = node->executeTick();
        rclcpp::spin_some(toolNode);
    }

    return status; //will return RUNNING if the while timed out
}

TEST_F(TestSetPose, test_ResetOdom_success) {
    //configure service
    auto resp = std::make_shared<SetPose::Response>();
    configSrv("/talos/set_pose", resp, 125ms);

    //test node, collect results
    BT::NodeStatus stat = testResetOdom(toolNode, 0, 0, 0, 0, 0, 0);
    auto req = std::make_shared<SetPose::Request>();
    bool reqReceived = killSrvAndGetRequest(req);

    //evaluate results
    ASSERT_EQ(stat, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(reqReceived);

    //check position
    ASSERT_EQ(req->pose.pose.pose.position.x, 0);
    ASSERT_EQ(req->pose.pose.pose.position.y, 0);
    ASSERT_EQ(req->pose.pose.pose.position.z, 0);

    //check orientation. I'm checking the quaternion because if I check rpy I'm restricted on orientation values I
    //can use for the test (conversion back to rpy can yield different but equivalent values)
    geometry_msgs::msg::Quaternion actualQuat = req->pose.pose.pose.orientation;
    ASSERT_NEAR(actualQuat.x, 0, TESTRESETODOM_EPSILON);
    ASSERT_NEAR(actualQuat.y, 0, TESTRESETODOM_EPSILON);
    ASSERT_NEAR(actualQuat.z, 0, TESTRESETODOM_EPSILON);
    ASSERT_NEAR(actualQuat.w, 1, TESTRESETODOM_EPSILON);
}

TEST_F(TestSetPose, test_ResetOdom_success_different_vals) {
    //configure service
    auto resp = std::make_shared<SetPose::Response>();
    configSrv("/talos/set_pose", resp, 125ms);

    //test node, collect results
    BT::NodeStatus stat = testResetOdom(toolNode, 3, 9.2, -1, 0, 0.1, 0.6);
    auto req = std::make_shared<SetPose::Request>();
    bool reqReceived = killSrvAndGetRequest(req);

    //evaluate results
    ASSERT_EQ(stat, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(reqReceived);

    //check position
    ASSERT_EQ(req->pose.pose.pose.position.x, 3);
    ASSERT_EQ(req->pose.pose.pose.position.y, 9.2);
    ASSERT_EQ(req->pose.pose.pose.position.z, -1);

    //check orientation. I'm checking the quaternion because if I check rpy I'm restricted on orientation values I
    //can use for the test (conversion back to rpy can yield different but equivalent values)
    geometry_msgs::msg::Vector3 expectedRpy;
    expectedRpy.x = 0;
    expectedRpy.y = 0.1;
    expectedRpy.z = 0.6;

    geometry_msgs::msg::Quaternion expectedQuat = toQuat(expectedRpy);
    geometry_msgs::msg::Quaternion actualQuat = req->pose.pose.pose.orientation;

    ASSERT_NEAR(actualQuat.x, expectedQuat.x, TESTRESETODOM_EPSILON);
    ASSERT_NEAR(actualQuat.y, expectedQuat.y, TESTRESETODOM_EPSILON);
    ASSERT_NEAR(actualQuat.z, expectedQuat.z, TESTRESETODOM_EPSILON);
    ASSERT_NEAR(actualQuat.w, expectedQuat.w, TESTRESETODOM_EPSILON);
}

TEST_F(TestSetPose, test_ResetOdom_fail_unavailable) {
    BT::NodeStatus stat = testResetOdom(toolNode, 1, 2, 3, 4, 5, 6);
    ASSERT_EQ(stat, BT::NodeStatus::FAILURE);
    ASSERT_GT(testElapsed(), 1s);
}
