#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/DummyService.hpp"

using namespace std::chrono_literals;
using MappingTarget = riptide_msgs2::srv::MappingTarget;
using MappingTargetTest = ServiceTest<MappingTarget>;

const std::chrono::duration<double> TESTSETMAPPINGTARGET_TIMEOUT = 3s;

BT::NodeStatus testSetMappingTargetService(std::shared_ptr<BtTestTool> toolNode, const std::string& targetObject, bool lock, double timeLimitSeconds) {
    //create node to test
    BT::NodeConfiguration cfg;
    cfg.input_ports["target_object"] = targetObject;
    cfg.input_ports["lock_map"] = (lock ? "1" : "0");
    cfg.input_ports["time_limit_secs"] = std::to_string(timeLimitSeconds);

    auto node = toolNode->createLeafNodeFromConfig("SetMappingTarget", cfg);

    //run the node to completion
    rclcpp::Time startTime = toolNode->get_clock()->now();
    return toolNode->tickUntilFinished(node, TESTSETMAPPINGTARGET_TIMEOUT);
}

TEST_F(MappingTargetTest, test_SetMappingTarget_success_some_object_no_lock) {
    //configure service
    auto resp = std::make_shared<MappingTarget::Response>();
    configSrv(MAPPING_SERVER_NAME, resp, 125ms);

    BT::NodeStatus stat = testSetMappingTargetService(toolNode, "some_object", false, 1);
    auto req = std::make_shared<MappingTarget::Request>();
    bool reqReceived = killSrvAndGetRequest(req);

    ASSERT_EQ(stat, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(reqReceived);
    ASSERT_EQ(req->target_object, "some_object");
    ASSERT_FALSE(req->lock_map);
}

TEST_F(MappingTargetTest, test_SetMappingTarget_success_some_object_lock) {
    //configure service
    auto resp = std::make_shared<MappingTarget::Response>();
    configSrv(MAPPING_SERVER_NAME, resp, 125ms);

    BT::NodeStatus stat = testSetMappingTargetService(toolNode, "some_object", true, 1);
    auto req = std::make_shared<MappingTarget::Request>();
    bool reqReceived = killSrvAndGetRequest(req);

    ASSERT_EQ(stat, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(reqReceived);
    ASSERT_EQ(req->target_object, "some_object");
    ASSERT_TRUE(req->lock_map);
}

TEST_F(MappingTargetTest, test_SetMappingTarget_success_other_object_lock) {
    //configure service
    auto resp = std::make_shared<MappingTarget::Response>();
    configSrv(MAPPING_SERVER_NAME, resp, 125ms);

    BT::NodeStatus stat = testSetMappingTargetService(toolNode, "other_object", false, 1);
    auto req = std::make_shared<MappingTarget::Request>();
    bool reqReceived = killSrvAndGetRequest(req);

    ASSERT_EQ(stat, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(reqReceived);
    ASSERT_EQ(req->target_object, "other_object");
    ASSERT_FALSE(req->lock_map);
}

TEST_F(MappingTargetTest, test_SetMappingTarget_success_other_object_lock_long_time) {
    //configure service
    auto resp = std::make_shared<MappingTarget::Response>();
    configSrv(MAPPING_SERVER_NAME, resp, 1500ms);

    BT::NodeStatus stat = testSetMappingTargetService(toolNode, "other_object", false, 4);
    auto req = std::make_shared<MappingTarget::Request>();
    bool reqReceived = killSrvAndGetRequest(req);

    ASSERT_EQ(stat, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(reqReceived);
    ASSERT_EQ(req->target_object, "other_object");
    ASSERT_FALSE(req->lock_map);
}

TEST_F(MappingTargetTest, test_SetMappingTarget_fail_not_available) {
    BT::NodeStatus stat = testSetMappingTargetService(toolNode, "some_object", false, 1);
    ASSERT_EQ(stat, BT::NodeStatus::FAILURE);
}

TEST_F(MappingTargetTest, test_SetMappingTarget_fail_timed_out) {
    //configure service
    auto resp = std::make_shared<MappingTarget::Response>();
    configSrv(MAPPING_SERVER_NAME, resp, 2s);

    BT::NodeStatus stat = testSetMappingTargetService(toolNode, "some_object", false, 1);
    auto req = std::make_shared<MappingTarget::Request>();
    bool reqReceived = killSrvAndGetRequest(req);

    ASSERT_EQ(stat, BT::NodeStatus::FAILURE);
    ASSERT_TRUE(reqReceived);
    ASSERT_EQ(req->target_object, "some_object");
    ASSERT_FALSE(req->lock_map);
}
