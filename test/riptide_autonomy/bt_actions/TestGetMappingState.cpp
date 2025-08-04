#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/TimedPublisher.hpp"

using namespace std::chrono_literals;

const std::chrono::duration<double> TESTGETMAPPINGSTATE_TIMEOUT = 5s;

BT::NodeStatus testGetMappingStateTopic(
    std::shared_ptr<BtTestTool> toolNode,
    const riptide_msgs2::msg::MappingTargetInfo valToPub,
    bool& outputSet,
    riptide_msgs2::msg::MappingTargetInfo& receivedVal,
    const int pubPeriodMs = 125)
{    
    //set up node
    BT::NodeConfiguration cfg;
    
    auto node = toolNode->createLeafNodeFromConfig("GetMappingState", cfg);
    
    //set up timed pub and expected msg
    TimedPublisher<riptide_msgs2::msg::MappingTargetInfo> timedPub(toolNode, MAPPING_TARGET_INFO_TOPIC, valToPub, 10, pubPeriodMs);

    //run node
    BT::NodeStatus status = toolNode->tickUntilFinished(node, TESTGETMAPPINGSTATE_TIMEOUT);

    //node done, get result
    outputSet = getOutputFromBlackboard<bool>(toolNode, node->config().blackboard, "map_locked", receivedVal.lock_map);
    outputSet = outputSet && getOutputFromBlackboard<std::string>(toolNode, node->config().blackboard, "target_name", receivedVal.target_object);
    return status;
}

TEST_F(BtTest, test_GetMappingState_not_locked_no_object) {
    bool outSet;
    riptide_msgs2::msg::MappingTargetInfo 
        val,
        result;
    
    val.lock_map = false;
    val.target_object = "";
    BT::NodeStatus stat = testGetMappingStateTopic(toolNode, val, outSet, result);

    ASSERT_EQ(stat, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(outSet);
    ASSERT_FALSE(result.lock_map);
    ASSERT_EQ(result.target_object, "");
}

TEST_F(BtTest, test_GetMappingState_locked_no_object) {
    bool outSet;
    riptide_msgs2::msg::MappingTargetInfo 
        val,
        result;
    
    val.lock_map = true;
    val.target_object = "";
    BT::NodeStatus stat = testGetMappingStateTopic(toolNode, val, outSet, result);

    ASSERT_EQ(stat, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(outSet);
    ASSERT_TRUE(result.lock_map);
    ASSERT_EQ(result.target_object, "");
}

TEST_F(BtTest, test_GetMappingState_not_locked_with_object) {
    bool outSet;
    riptide_msgs2::msg::MappingTargetInfo 
        val,
        result;
    
    val.lock_map = false;
    val.target_object = "something";
    BT::NodeStatus stat = testGetMappingStateTopic(toolNode, val, outSet, result);

    ASSERT_EQ(stat, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(outSet);
    ASSERT_FALSE(result.lock_map);
    ASSERT_EQ(result.target_object, "something");
}

TEST_F(BtTest, test_GetMappingState_locked_with_object) {
    bool outSet;
    riptide_msgs2::msg::MappingTargetInfo 
        val,
        result;
    
    val.lock_map = true;
    val.target_object = "something";
    BT::NodeStatus stat = testGetMappingStateTopic(toolNode, val, outSet, result);

    ASSERT_EQ(stat, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(outSet);
    ASSERT_TRUE(result.lock_map);
    ASSERT_EQ(result.target_object, "something");
}

TEST_F(BtTest, test_GetMappingState_fail_timeout) {
    bool outSet;
    riptide_msgs2::msg::MappingTargetInfo 
        val,
        result;
    
    val.lock_map = true;
    val.target_object = "stuff";
    
    BT::NodeStatus stat = testGetMappingStateTopic(toolNode, val, outSet, result, 3500);

    ASSERT_EQ(stat, BT::NodeStatus::FAILURE);
    ASSERT_TRUE(outSet);
    ASSERT_FALSE(result.lock_map);
    ASSERT_EQ(result.target_object, "");
}
