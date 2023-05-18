#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/DummyService.hpp"

using namespace std::chrono_literals;
using Trigger = std_srvs::srv::Trigger;
using TriggerTest = ServiceTest<Trigger>;

const std::chrono::duration<double> TESTCALLTRIGGERSRV_TIMEOUT = 5s;

BT::NodeStatus testCallTriggerService(std::shared_ptr<BtTestTool> toolNode, const std::string& srvName, double nodeTimeLimitSecs) {
    //create the BT node
    BT::NodeConfiguration cfg;
    cfg.input_ports["srv_name"] = srvName;
    cfg.input_ports["time_limit_secs"] = std::to_string(nodeTimeLimitSecs);

    auto node = toolNode->createLeafNodeFromConfig("CallTriggerService", cfg);

    //run the node
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    rclcpp::Time startTime = toolNode->get_clock()->now();
    while(status != BT::NodeStatus::SUCCESS && status != BT::NodeStatus::FAILURE && toolNode->get_clock()->now() - startTime < TESTCALLTRIGGERSRV_TIMEOUT) {
        status = node->executeTick();
        rclcpp::spin_some(toolNode);
    }

    return status; //will return RUNNING if the while timed out
}

TEST_F(TriggerTest, test_CallTriggerService_success) {
    //set up server
    auto resp = std::make_shared<Trigger::Response>();
    resp->success = true; //node should return SUCCESS
    configSrv("/some_trigger_srv", resp, 125ms);

    //test trigger service and collect results
    BT::NodeStatus stat = testCallTriggerService(toolNode, "/some_trigger_srv", 1);
    auto req = std::make_shared<Trigger::Request>();
    bool reqReceived = killSrvAndGetRequest(req);

    //eval results
    ASSERT_EQ(stat, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(reqReceived);
}

TEST_F(TriggerTest, test_CallTriggerService_success_another_topic) {
    //set up server
    auto resp = std::make_shared<Trigger::Response>();
    resp->success = true; //node should return SUCCESS
    configSrv("yet_another_trigger_srv", resp, 125ms);

    //test trigger service and collect results
    BT::NodeStatus stat = testCallTriggerService(toolNode, "yet_another_trigger_srv", 1);
    auto req = std::make_shared<Trigger::Request>();
    bool reqReceived = killSrvAndGetRequest(req);

    //eval results
    ASSERT_EQ(stat, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(reqReceived);
}

TEST_F(TriggerTest, test_CallTriggerService_fail_result_unsuccessful) {
    //set up server
    auto resp = std::make_shared<Trigger::Response>();
    resp->success = false; //node should return FAILURE
    configSrv("yet_another_trigger_srv", resp, 125ms);

    //test trigger service and collect results
    BT::NodeStatus stat = testCallTriggerService(toolNode, "yet_another_trigger_srv", 1);
    auto req = std::make_shared<Trigger::Request>();
    bool reqReceived = killSrvAndGetRequest(req);

    //eval results
    ASSERT_EQ(stat, BT::NodeStatus::FAILURE);
    ASSERT_TRUE(reqReceived);
}

TEST_F(TriggerTest, test_CallTriggerService_fail_timed_out) {
    //set up server
    auto resp = std::make_shared<Trigger::Response>();
    resp->success = true; //node should return SUCCESS (but itll time out and return FAILURE)
    configSrv("/some_trigger_srv", resp, 1s);

    //test trigger service and collect results
    BT::NodeStatus stat = testCallTriggerService(toolNode, "/some_trigger_srv", 0.5);
    auto req = std::make_shared<Trigger::Request>();
    bool reqReceived = killSrvAndGetRequest(req);

    //eval results
    ASSERT_EQ(stat, BT::NodeStatus::FAILURE);
    ASSERT_TRUE(reqReceived);
}

TEST_F(TriggerTest, test_CallTriggerService_fail_unavailable) {
    BT::NodeStatus stat = testCallTriggerService(toolNode, "/some_trigger_srv", 1);
    
    ASSERT_EQ(stat, BT::NodeStatus::FAILURE);
    ASSERT_GT(testElapsed(), 1s);
}
