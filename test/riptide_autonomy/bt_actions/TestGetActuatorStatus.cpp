#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/TimedPublisher.hpp"

struct GetActuatorStatusInfo {
    int
        statusIntervalMs,
        busyIntervalMs;
    
    int
        clawState,
        torpedoState,
        torpedoAvailable,
        dropperState,
        dropperAvailable;
    
    bool busy;
};

BT::NodeStatus testGetActuatorStatus(std::shared_ptr<BtTestTool> toolNode, GetActuatorStatusInfo info, GetActuatorStatusInfo& out, bool& outputsSet) {
    //configure node
    BT::NodeConfiguration cfg;
    auto node = toolNode->createLeafNodeFromConfig("GetActuatorStatus", cfg);

    //configure publishers
    riptide_msgs2::msg::ActuatorStatus actStatus;
    actStatus.claw_state = info.clawState;
    actStatus.torpedo_state = info.torpedoState;
    actStatus.torpedo_available_count = info.torpedoAvailable;
    actStatus.dropper_state = info.dropperState;
    actStatus.dropper_available_count = info.dropperAvailable;
    TimedPublisher<riptide_msgs2::msg::ActuatorStatus> statusPub(toolNode, ACTUATOR_STATUS_TOPIC, actStatus, 10, info.statusIntervalMs);
    
    std_msgs::msg::Bool busy;
    busy.data = info.busy;
    TimedPublisher<std_msgs::msg::Bool> busyPub(toolNode, ACTUATOR_BUSY_TOPIC, busy, 10, info.busyIntervalMs);

    //tick until completed
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    rclcpp::Time startTime = toolNode->get_clock()->now();
    while(status != BT::NodeStatus::SUCCESS && status != BT::NodeStatus::FAILURE && toolNode->get_clock()->now() - startTime < 5s) {
        status = node->executeTick();
        rclcpp::spin_some(toolNode);
    }
    
    outputsSet = true;
    auto blackboard = node->config().blackboard;

    outputsSet = getFromBlackboard<int>(blackboard, "claw_state", out.clawState) && outputsSet;
    outputsSet = getFromBlackboard<int>(blackboard, "torpedo_state", out.torpedoState) && outputsSet;
    outputsSet = getFromBlackboard<int>(blackboard, "torpedo_available_count", out.torpedoAvailable) && outputsSet;
    outputsSet = getFromBlackboard<int>(blackboard, "dropper_state", out.dropperState) && outputsSet;
    outputsSet = getFromBlackboard<int>(blackboard, "dropper_available_count", out.dropperAvailable) && outputsSet;
    outputsSet = getFromBlackboard<bool>(blackboard, "actuators_busy", out.busy) && outputsSet;

    return status;
}


TEST_F(BtTest, test_GetActuatorStatus_success_busy) {
    //setup
    GetActuatorStatusInfo in;
    in.statusIntervalMs     = 50;
    in.busyIntervalMs       = 50;
    in.clawState            = riptide_msgs2::msg::ActuatorStatus::CLAW_CLOSED;
    in.torpedoState         = riptide_msgs2::msg::ActuatorStatus::TORPEDO_DISARMED;
    in.torpedoAvailable     = 2;
    in.dropperState         = riptide_msgs2::msg::ActuatorStatus::DROPPER_DISARMED;
    in.dropperAvailable     = 2;
    in.busy                 = true;

    //perform test
    GetActuatorStatusInfo out;
    bool outputsSet = false;
    BT::NodeStatus status = testGetActuatorStatus(toolNode, in, out, outputsSet);

    //eval result
    ASSERT_TRUE(outputsSet);
    ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(out.clawState, in.clawState);
    ASSERT_EQ(out.torpedoState, in.torpedoState);
    ASSERT_EQ(out.torpedoAvailable, in.torpedoAvailable);
    ASSERT_EQ(out.dropperState, in.dropperState);
    ASSERT_EQ(out.dropperAvailable, in.dropperAvailable);
    ASSERT_EQ(out.busy, in.busy);
}

TEST_F(BtTest, test_GetActuatorStatus_success_not_busy) {
    //setup
    GetActuatorStatusInfo in;
    in.statusIntervalMs     = 250;
    in.busyIntervalMs       = 1000;
    in.clawState            = riptide_msgs2::msg::ActuatorStatus::CLAW_OPENED;
    in.torpedoState         = riptide_msgs2::msg::ActuatorStatus::TORPEDO_FIRING;
    in.torpedoAvailable     = 2;
    in.dropperState         = riptide_msgs2::msg::ActuatorStatus::DROPPER_READY;
    in.dropperAvailable     = 1;
    in.busy                 = false;

    //perform test
    GetActuatorStatusInfo out;
    bool outputsSet = false;
    BT::NodeStatus status = testGetActuatorStatus(toolNode, in, out, outputsSet);

    //eval result
    ASSERT_TRUE(outputsSet);
    ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(out.clawState, in.clawState);
    ASSERT_EQ(out.torpedoState, in.torpedoState);
    ASSERT_EQ(out.torpedoAvailable, in.torpedoAvailable);
    ASSERT_EQ(out.dropperState, in.dropperState);
    ASSERT_EQ(out.dropperAvailable, in.dropperAvailable);
    ASSERT_EQ(out.busy, in.busy);
}

TEST_F(BtTest, test_GetActuatorStatus_success_one_more_time) {
    //setup
    GetActuatorStatusInfo in;
    in.statusIntervalMs     = 1000;
    in.busyIntervalMs       = 250;
    in.clawState            = riptide_msgs2::msg::ActuatorStatus::CLAW_OPENED;
    in.torpedoState         = riptide_msgs2::msg::ActuatorStatus::TORPEDO_FIRING;
    in.torpedoAvailable     = 0;
    in.dropperState         = riptide_msgs2::msg::ActuatorStatus::DROPPER_READY;
    in.dropperAvailable     = 1;
    in.busy                 = false;

    //perform test
    GetActuatorStatusInfo out;
    bool outputsSet = false;
    BT::NodeStatus status = testGetActuatorStatus(toolNode, in, out, outputsSet);

    //eval result
    ASSERT_TRUE(outputsSet);
    ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(out.clawState, in.clawState);
    ASSERT_EQ(out.torpedoState, in.torpedoState);
    ASSERT_EQ(out.torpedoAvailable, in.torpedoAvailable);
    ASSERT_EQ(out.dropperState, in.dropperState);
    ASSERT_EQ(out.dropperAvailable, in.dropperAvailable);
    ASSERT_EQ(out.busy, in.busy);
}

TEST_F(BtTest, test_GetActuatorStatus_fail_timed_out_no_publish) {
    //setup
    GetActuatorStatusInfo in;
    in.statusIntervalMs     = 5000;
    in.busyIntervalMs       = 5000;
    in.clawState            = riptide_msgs2::msg::ActuatorStatus::CLAW_OPENED;
    in.torpedoState         = riptide_msgs2::msg::ActuatorStatus::TORPEDO_FIRING;
    in.torpedoAvailable     = 0;
    in.dropperState         = riptide_msgs2::msg::ActuatorStatus::DROPPER_READY;
    in.dropperAvailable     = 1;
    in.busy                 = false;

    //perform test
    GetActuatorStatusInfo out;
    bool outputsSet = false;
    BT::NodeStatus status = testGetActuatorStatus(toolNode, in, out, outputsSet);

    //eval result
    ASSERT_FALSE(outputsSet);
    ASSERT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(BtTest, test_GetActuatorStatus_fail_timed_out_publish_only_busy) {
    //setup
    GetActuatorStatusInfo in;
    in.statusIntervalMs     = 5000;
    in.busyIntervalMs       = 100;
    in.clawState            = riptide_msgs2::msg::ActuatorStatus::CLAW_OPENED;
    in.torpedoState         = riptide_msgs2::msg::ActuatorStatus::TORPEDO_FIRING;
    in.torpedoAvailable     = 0;
    in.dropperState         = riptide_msgs2::msg::ActuatorStatus::DROPPER_READY;
    in.dropperAvailable     = 1;
    in.busy                 = false;

    //perform test
    GetActuatorStatusInfo out;
    bool outputsSet = false;
    BT::NodeStatus status = testGetActuatorStatus(toolNode, in, out, outputsSet);

    //eval result
    ASSERT_FALSE(outputsSet);
    ASSERT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(BtTest, test_GetActuatorStatus_fail_timed_out_publish_only_status) {
    //setup
    GetActuatorStatusInfo in;
    in.statusIntervalMs     = 100;
    in.busyIntervalMs       = 5000;
    in.clawState            = riptide_msgs2::msg::ActuatorStatus::CLAW_OPENED;
    in.torpedoState         = riptide_msgs2::msg::ActuatorStatus::TORPEDO_FIRING;
    in.torpedoAvailable     = 0;
    in.dropperState         = riptide_msgs2::msg::ActuatorStatus::DROPPER_READY;
    in.dropperAvailable     = 1;
    in.busy                 = false;

    //perform test
    GetActuatorStatusInfo out;
    bool outputsSet = false;
    BT::NodeStatus status = testGetActuatorStatus(toolNode, in, out, outputsSet);

    //eval result
    ASSERT_FALSE(outputsSet);
    ASSERT_EQ(status, BT::NodeStatus::FAILURE);
}
