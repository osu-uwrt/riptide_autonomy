#include "autonomy_test/autonomy_testing.hpp"

#define PRINT_RESULTS_OF_TEST

//
// TEST UTIL THINGS
//

struct DualVector3 {
    geometry_msgs::msg::Vector3
        v1,
        v2;
};

#define ASSERT_LINKS_EQ(node, link1, link2, absError) \
    do { \
        geometry_msgs::msg::TransformStamped __transform; \
        bool __evalTransformLookedUp = lookupTransform(node, link1, link2, __transform); \
        ASSERT_TRUE(__evalTransformLookedUp); \
        geometry_msgs::msg::Vector3 __transRpy = toRPY(__transform.transform.rotation); \
        ASSERT_NEAR(__transform.transform.translation.x, 0, absError); \
        ASSERT_NEAR(__transform.transform.translation.y, 0, absError); \
        ASSERT_NEAR(__transform.transform.translation.z, 0, absError); \
        ASSERT_NEAR(__transRpy.x, 0, absError); \
        ASSERT_NEAR(__transRpy.y, 0, absError); \
        ASSERT_NEAR(__transRpy.z, 0, absError); \
    } while(0)


inline geometry_msgs::msg::Vector3 makeVector3(double x, double y, double z) {
    geometry_msgs::msg::Vector3 v;
    v.x = x;
    v.y = y;
    v.z = z;
    return v;
}    


inline DualVector3 makeDualVector3(double v1x, double v1y, double v1z, double v2x, double v2y, double v2z) {
    DualVector3 dv3;
    dv3.v1 = makeVector3(v1x, v1y, v1z);
    dv3.v2 = makeVector3(v2x, v2y, v2z);
    return dv3;
}


geometry_msgs::msg::TransformStamped createTransform(std::shared_ptr<BtTestTool> toolNode, DualVector3 pose, std::string parentFrame, std::string childFrame) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = toolNode->get_clock()->now();

    transform.transform.translation = pose.v1;
    
    geometry_msgs::msg::Vector3 transformRPY = pose.v2;
    transform.transform.rotation = toQuat(transformRPY);

    transform.header.frame_id = parentFrame;
    transform.child_frame_id = childFrame;

    return transform;
}


bool lookupTransform(rclcpp::Node::SharedPtr node, const std::string& fromFrame, const std::string& toFrame, geometry_msgs::msg::TransformStamped& transform) {
    bool evalTransformLookedUp = false;
    std::shared_ptr<tf2_ros::Buffer> buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf2_ros::TransformListener listener(*buffer);
    DEF_THROTTLE_TIMER(throtTimer);
    while(!evalTransformLookedUp) {
        evalTransformLookedUp = lookupTransformThrottled(node, buffer, fromFrame, toFrame, 0.5, throtTimer, transform, true);
    }

    return evalTransformLookedUp;
}



BT::NodeStatus testLinkAlign(
    std::shared_ptr<BtTestTool> toolNode,
    tf2_ros::StaticTransformBroadcaster& tfBroadcaster,
    const DualVector3& baseLinkPose,
    const DualVector3& childLinkPose,
    const std::string& childLinkName,
    const DualVector3& linkGoalPose,
    const DualVector3& goalFramePose,
    const std::string& goalFrameName,
    bool& outputsSet,
    bool publishBaseToChild = true) 
{
    std::vector<geometry_msgs::msg::TransformStamped> transforms = {
        createTransform(toolNode, baseLinkPose, "world", "bt_testing/base_link"),
        createTransform(toolNode, linkGoalPose, "world", "bt_testing/testlinkgoal"),
        createTransform(toolNode, goalFramePose, "world", goalFrameName)
    };

    if(publishBaseToChild) {
        transforms.push_back(
            createTransform(toolNode, childLinkPose, "bt_testing/base_link", childLinkName)
        );
    }

    tfBroadcaster.sendTransform(transforms);

    BT::NodeConfiguration cfg;
    cfg.input_ports["x"] = std::to_string(linkGoalPose.v1.x);
    cfg.input_ports["y"] = std::to_string(linkGoalPose.v1.y);
    cfg.input_ports["z"] = std::to_string(linkGoalPose.v1.z);
    cfg.input_ports["or"] = std::to_string(linkGoalPose.v2.x);
    cfg.input_ports["op"] = std::to_string(linkGoalPose.v2.y);
    cfg.input_ports["oy"] = std::to_string(linkGoalPose.v2.z);
    cfg.input_ports["reference_frame"] = goalFrameName;
    cfg.input_ports["target_frame"] = childLinkName;

    auto node = toolNode->createLeafNodeFromConfig("ComputeFrameAlignment", cfg);
    BT::NodeStatus status = toolNode->tickUntilFinished(node);

    outputsSet = true;
    BT::Blackboard::Ptr bb = node->config().blackboard;
    DualVector3 baseLinkGoal;
    outputsSet = outputsSet && getOutputFromBlackboard<double>(toolNode, bb, "out_x", baseLinkGoal.v1.x);
    outputsSet = outputsSet && getOutputFromBlackboard<double>(toolNode, bb, "out_y", baseLinkGoal.v1.y);
    outputsSet = outputsSet && getOutputFromBlackboard<double>(toolNode, bb, "out_z", baseLinkGoal.v1.z);
    outputsSet = outputsSet && getOutputFromBlackboard<double>(toolNode, bb, "out_or", baseLinkGoal.v2.x);
    outputsSet = outputsSet && getOutputFromBlackboard<double>(toolNode, bb, "out_op", baseLinkGoal.v2.y);
    outputsSet = outputsSet && getOutputFromBlackboard<double>(toolNode, bb, "out_oy", baseLinkGoal.v2.z);

    //publish outputs
    tfBroadcaster.sendTransform({
        createTransform(toolNode, baseLinkGoal, "world", "eval_base_link"),
        createTransform(toolNode, childLinkPose, "eval_base_link", "eval_link")
    });


    #ifdef PRINT_RESULTS_OF_TEST
        RCLCPP_INFO(toolNode->get_logger(), "bl goal is %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
            baseLinkGoal.v1.x,
            baseLinkGoal.v1.y,
            baseLinkGoal.v1.z,
            baseLinkGoal.v2.x,
            baseLinkGoal.v2.y,
            baseLinkGoal.v2.z);
    #endif

    return status;
}

TEST_F(BtTest, test_ComputeFrameAlignment_exactly_baselink_to_zero) {
    tf2_ros::StaticTransformBroadcaster broadcaster(toolNode);
    bool outputsSet;

    DualVector3
        baseLinkPose    = makeDualVector3(0, 0, 0, 0, 0, 0),
        childLinkPose   = makeDualVector3(0, 0, 0, 0, 0, 0),
        childLinkGoal   = makeDualVector3(0, 0, 0, 0, 0, 0),
        goalFramePose   = makeDualVector3(0, 0, 0, 0, 0, 0);
    
    BT::NodeStatus status = testLinkAlign(
        toolNode,
        broadcaster,
        baseLinkPose,
        childLinkPose,
        "some_child_link",
        childLinkGoal,
        goalFramePose,
        "some_goal_frame",
        outputsSet
    );

    ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(outputsSet);
    ASSERT_LINKS_EQ(toolNode, "eval_link", "bt_testing/testlinkgoal", 0.01);
}

TEST_F(BtTest, test_ComputeFrameAlignment_offset_child_without_rotation_to_zero) {
    tf2_ros::StaticTransformBroadcaster broadcaster(toolNode);
    bool outputsSet;

    DualVector3
        baseLinkPose    = makeDualVector3(0, 0, 0, 0, 0, 0),
        childLinkPose   = makeDualVector3(1, 2, 3, 0, 0, 0),
        childLinkGoal   = makeDualVector3(0, 0, 0, 0, 0, 0),
        goalFramePose   = makeDualVector3(0, 0, 0, 0, 0, 0);
    
    BT::NodeStatus status = testLinkAlign(
        toolNode,
        broadcaster,
        baseLinkPose,
        childLinkPose,
        "some_child_link",
        childLinkGoal,
        goalFramePose,
        "some_goal_frame",
        outputsSet
    );

    ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(outputsSet);
    ASSERT_LINKS_EQ(toolNode, "eval_link", "bt_testing/testlinkgoal", 0.01);
}

TEST_F(BtTest, test_ComputeFrameAlignment_offset_child_with_rotation_to_zero) {
    tf2_ros::StaticTransformBroadcaster broadcaster(toolNode);
    bool outputsSet;

    DualVector3
        baseLinkPose    = makeDualVector3(0, 0, 0, 0, 0, 0),
        childLinkPose   = makeDualVector3(1, 2, 3, 0, 0, 3.1415), //TODO make weirder
        childLinkGoal   = makeDualVector3(0, 0, 0, 0, 0, 0),
        goalFramePose   = makeDualVector3(0, 0, 0, 0, 0, 0);
    
    BT::NodeStatus status = testLinkAlign(
        toolNode,
        broadcaster,
        baseLinkPose,
        childLinkPose,
        "some_child_link",
        childLinkGoal,
        goalFramePose,
        "some_goal_frame",
        outputsSet
    );

    ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(outputsSet);
    ASSERT_LINKS_EQ(toolNode, "eval_link", "bt_testing/testlinkgoal", 0.01);
}

TEST_F(BtTest, test_ComputeFrameAlignment_offset_child_without_rotation_to_nonzero_without_rotation) {
    tf2_ros::StaticTransformBroadcaster broadcaster(toolNode);
    bool outputsSet;

    DualVector3
        baseLinkPose    = makeDualVector3(0, 0, 0, 0, 0, 0),
        childLinkPose   = makeDualVector3(1, 2, 3, 0, 0, 0),
        childLinkGoal   = makeDualVector3(9, 8, 7, 0, 0, 0),
        goalFramePose   = makeDualVector3(0, 0, 0, 0, 0, 0);
    
    BT::NodeStatus status = testLinkAlign(
        toolNode,
        broadcaster,
        baseLinkPose,
        childLinkPose,
        "some_child_link",
        childLinkGoal,
        goalFramePose,
        "some_goal_frame",
        outputsSet
    );

    ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(outputsSet);
    ASSERT_LINKS_EQ(toolNode, "eval_link", "bt_testing/testlinkgoal", 0.01);
}

TEST_F(BtTest, test_ComputeFrameAlignment_offset_child_without_rotation_to_nonzero_with_rotation) {
    tf2_ros::StaticTransformBroadcaster broadcaster(toolNode);
    bool outputsSet;

    DualVector3
        baseLinkPose    = makeDualVector3(0, 0, 0, 0, 0, 0),
        childLinkPose   = makeDualVector3(1, 2, 3, 0, 0, 0),
        childLinkGoal   = makeDualVector3(9, 8, 7, 4, 5, 6),
        goalFramePose   = makeDualVector3(0, 0, 0, 0, 0, 0);
    
    BT::NodeStatus status = testLinkAlign(
        toolNode,
        broadcaster,
        baseLinkPose,
        childLinkPose,
        "some_child_link",
        childLinkGoal,
        goalFramePose,
        "some_goal_frame",
        outputsSet
    );

    ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(outputsSet);
    ASSERT_LINKS_EQ(toolNode, "eval_link", "bt_testing/testlinkgoal", 0.01);
}

TEST_F(BtTest, test_ComputeFrameAlignment_offset_child_with_rotation_to_nonzero_with_rotation) {
    tf2_ros::StaticTransformBroadcaster broadcaster(toolNode);
    bool outputsSet;

    DualVector3
        baseLinkPose    = makeDualVector3(0, 0, 0, 0, 0, 0),
        childLinkPose   = makeDualVector3(1, 2, 3, 1, 4, 2),
        childLinkGoal   = makeDualVector3(9, 8, 7, 4, 5, 6),
        goalFramePose   = makeDualVector3(0, 0, 0, 0, 0, 0);
    
    BT::NodeStatus status = testLinkAlign(
        toolNode,
        broadcaster,
        baseLinkPose,
        childLinkPose,
        "some_child_link",
        childLinkGoal,
        goalFramePose,
        "some_goal_frame",
        outputsSet
    );

    ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(outputsSet);
    ASSERT_LINKS_EQ(toolNode, "eval_link", "bt_testing/testlinkgoal", 0.01);
}

TEST_F(BtTest, test_ComputeFrameAlignment_offset_child_without_rotation_to_nonzero_without_rotation_offset_goal_without_rotation) {
    tf2_ros::StaticTransformBroadcaster broadcaster(toolNode);
    bool outputsSet;

    DualVector3
        baseLinkPose    = makeDualVector3(0, 0, 0, 0, 0, 0),
        childLinkPose   = makeDualVector3(1, 2, 3, 0, 0, 0),
        childLinkGoal   = makeDualVector3(9, 8, 7, 0, 0, 0),
        goalFramePose   = makeDualVector3(7, 8, 9, 0, 0, 0);
    
    BT::NodeStatus status = testLinkAlign(
        toolNode,
        broadcaster,
        baseLinkPose,
        childLinkPose,
        "some_child_link",
        childLinkGoal,
        goalFramePose,
        "some_goal_frame",
        outputsSet
    );

    ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(outputsSet);
    ASSERT_LINKS_EQ(toolNode, "eval_link", "bt_testing/testlinkgoal", 0.01);
}

TEST_F(BtTest, test_ComputeFrameAlignment_offset_child_with_rotation_to_nonzero_with_rotation_offset_goal_without_rotation) {
    tf2_ros::StaticTransformBroadcaster broadcaster(toolNode);
    bool outputsSet;

    DualVector3
        baseLinkPose    = makeDualVector3(0, 0, 0, 0, 0, 0),
        childLinkPose   = makeDualVector3(1, 2, 3, 1, 4, 2),
        childLinkGoal   = makeDualVector3(9, 8, 7, 4, 5, 6),
        goalFramePose   = makeDualVector3(7, 8, 9, 0, 0, 0);
    
    BT::NodeStatus status = testLinkAlign(
        toolNode,
        broadcaster,
        baseLinkPose,
        childLinkPose,
        "some_child_link",
        childLinkGoal,
        goalFramePose,
        "some_goal_frame",
        outputsSet
    );

    ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(outputsSet);
    ASSERT_LINKS_EQ(toolNode, "eval_link", "bt_testing/testlinkgoal", 0.01);
}

TEST_F(BtTest, test_ComputeFrameAlignment_offset_child_with_rotation_to_nonzero_with_rotation_offset_goal_with_rotation) {
    tf2_ros::StaticTransformBroadcaster broadcaster(toolNode);
    bool outputsSet;

    DualVector3
        baseLinkPose    = makeDualVector3(0, 0, 0, 0, 0, 0),
        childLinkPose   = makeDualVector3(1, 2, 3, 1, 4, 2),
        childLinkGoal   = makeDualVector3(9, 8, 7, 4, 5, 6),
        goalFramePose   = makeDualVector3(7, 8, 9, 2, 3, 1);
    
    BT::NodeStatus status = testLinkAlign(
        toolNode,
        broadcaster,
        baseLinkPose,
        childLinkPose,
        "some_child_link",
        childLinkGoal,
        goalFramePose,
        "some_goal_frame",
        outputsSet
    );

    ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(outputsSet);
    ASSERT_LINKS_EQ(toolNode, "eval_link", "bt_testing/testlinkgoal", 0.01);
}

TEST_F(BtTest, test_ComputeFrameAlignment_offset_baselink_offset_child_with_rotation_to_nonzero_with_rotation_offset_goal_with_rotation) {
    tf2_ros::StaticTransformBroadcaster broadcaster(toolNode);
    bool outputsSet;

    DualVector3
        baseLinkPose    = makeDualVector3(9, 8, 7, 6, 5, 4),
        childLinkPose   = makeDualVector3(1, 2, 3, 1, 4, 2),
        childLinkGoal   = makeDualVector3(9, 8, 7, 4, 5, 6),
        goalFramePose   = makeDualVector3(7, 8, 9, 2, 3, 1);
    
    BT::NodeStatus status = testLinkAlign(
        toolNode,
        broadcaster,
        baseLinkPose,
        childLinkPose,
        "some_child_link",
        childLinkGoal,
        goalFramePose,
        "some_goal_frame",
        outputsSet
    );

    ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(outputsSet);
    ASSERT_LINKS_EQ(toolNode, "eval_link", "bt_testing/testlinkgoal", 0.01);
}
