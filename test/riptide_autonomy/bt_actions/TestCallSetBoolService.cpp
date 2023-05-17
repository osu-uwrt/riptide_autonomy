#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/DummyService.hpp"

using namespace std::chrono_literals;
using SetBool = std_srvs::srv::SetBool;
using SetBoolTest = ServiceTest<SetBool>;

const std::chrono::duration<double> TESTSETBOOLSRV_TIMEOUT = 5s;

BT::NodeStatus testCallSetBoolSrv(
    std::shared_ptr<BtTestTool> toolNode, 
    const std::string& srvName, 
    bool valToUse, 
    double nodeTimeLimitSecs)
{
    //create the node
    BT::NodeConfiguration cfg;
    cfg.input_ports["srv_name"] = srvName;
    cfg.input_ports["data"] = (valToUse ? "1" : "0");
    cfg.input_ports["time_limit_secs"] = std::to_string(nodeTimeLimitSecs);

    auto node = toolNode->createLeafNodeFromConfig("CallSetBoolService", cfg);

    BT::NodeStatus status = BT::NodeStatus::IDLE;
    rclcpp::Time startTime = toolNode->get_clock()->now();
    while(status != BT::NodeStatus::SUCCESS && status != BT::NodeStatus::FAILURE && toolNode->get_clock()->now() - startTime < TESTSETBOOLSRV_TIMEOUT) {
        status = node->executeTick();
        rclcpp::spin_some(toolNode);
    }

    return status; //if timed out, will return RUNNING
}


TEST_F(SetBoolTest, test_CallSetBoolService_true_success) {
    //configure service
    auto resp = std::make_shared<SetBool::Response>();
    resp->success = true; //node should return SUCCESS

    configSrv("/some_setbool_srv", resp, 125ms);

    BT::NodeStatus stat = testCallSetBoolSrv(toolNode, "/some_setbool_srv", true, 1);
    auto req = std::make_shared<SetBool::Request>();
    bool reqReceived = killSrvAndGetRequest(req);

    ASSERT_EQ(stat, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(reqReceived);
    ASSERT_TRUE(req->data);
}

TEST_F(SetBoolTest, test_CallSetBoolService_false_success) {
    //configure service
    auto resp = std::make_shared<SetBool::Response>();
    resp->success = true; //node should return SUCCESS

    configSrv("/some_setbool_srv", resp, 125ms);

    BT::NodeStatus stat = testCallSetBoolSrv(toolNode, "/some_setbool_srv", false, 1);
    auto req = std::make_shared<SetBool::Request>();
    bool reqReceived = killSrvAndGetRequest(req);

    ASSERT_EQ(stat, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(reqReceived);
    ASSERT_FALSE(req->data);
}

TEST_F(SetBoolTest, test_CallSetBoolService_true_success_other_name) {
    //configure service
    auto resp = std::make_shared<SetBool::Response>();
    resp->success = true; //node should return SUCCESS

    configSrv("another/setbool/service", resp, 125ms);

    BT::NodeStatus stat = testCallSetBoolSrv(toolNode, "another/setbool/service", true, 1);
    auto req = std::make_shared<SetBool::Request>();
    bool reqReceived = killSrvAndGetRequest(req);

    ASSERT_EQ(stat, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(reqReceived);
    ASSERT_TRUE(req->data);
}

TEST_F(SetBoolTest, test_CallSetBoolService_false_success_other_name) {
    //configure service
    auto resp = std::make_shared<SetBool::Response>();
    resp->success = true; //node should return SUCCESS

    configSrv("another/setbool/service", resp, 125ms);

    BT::NodeStatus stat = testCallSetBoolSrv(toolNode, "another/setbool/service", false, 1);
    auto req = std::make_shared<SetBool::Request>();
    bool reqReceived = killSrvAndGetRequest(req);

    ASSERT_EQ(stat, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(reqReceived);
    ASSERT_FALSE(req->data);
}

TEST_F(SetBoolTest, test_CallSetBoolService_fail_resp_unsuccessful) {
    //configure service
    auto resp = std::make_shared<SetBool::Response>();
    resp->success = false; //node should return FAILURE

    configSrv("/some_setbool_srv", resp, 125ms);

    BT::NodeStatus stat = testCallSetBoolSrv(toolNode, "/some_setbool_srv", false, 1);
    auto req = std::make_shared<SetBool::Request>();
    bool reqReceived = killSrvAndGetRequest(req);

    ASSERT_EQ(stat, BT::NodeStatus::FAILURE);
    ASSERT_TRUE(reqReceived);
    ASSERT_FALSE(req->data);
}

TEST_F(SetBoolTest, test_CallSetBoolService_fail_timeout) {
    //configure service
    auto resp = std::make_shared<SetBool::Response>();
    resp->success = true; //node should return SUCCESS (but it wont because the service will take to long)

    configSrv("/some_setbool_srv", resp, 2s);

    BT::NodeStatus stat = testCallSetBoolSrv(toolNode, "/some_setbool_srv", false, 1);
    auto req = std::make_shared<SetBool::Request>();
    bool reqReceived = killSrvAndGetRequest(req);

    ASSERT_EQ(stat, BT::NodeStatus::FAILURE);
    ASSERT_TRUE(reqReceived);
    ASSERT_FALSE(req->data);
}

TEST_F(SetBoolTest, test_CallSetBoolService_fail_not_available) {
    BT::NodeStatus stat = testCallSetBoolSrv(toolNode, "/some_setbool_srv", false, 1);
    ASSERT_EQ(stat, BT::NodeStatus::FAILURE);
    ASSERT_GT(testElapsed(), 1s);
}
