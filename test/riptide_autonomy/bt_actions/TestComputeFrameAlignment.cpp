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


#define ASSERT_LINK_NEAR_POS(node, linkName, pose, absError) \
    do { \
        geometry_msgs::msg::TransformStamped transform; \
        bool evalTransformLookedUp = lookupTransform(node, linkName, "world", transform); \
        ASSERT_TRUE(evalTransformLookedUp); \
        ASSERT_NEAR(transform.transform.translation.x, pose.v1.x, absError); \
        ASSERT_NEAR(transform.transform.translation.y, pose.v1.y, absError); \
        ASSERT_NEAR(transform.transform.translation.z, pose.v1.z, absError); \
        geometry_msgs::msg::Vector3 transRpy = toRPY(transform.transform.rotation); \
        ASSERT_NEAR(transRpy.x, pose.v2.x, absError); \
        ASSERT_NEAR(transRpy.y, pose.v2.y, absError); \
        ASSERT_NEAR(transRpy.z, pose.v2.z, absError); \
    } while(false)


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
        evalTransformLookedUp = lookupTransformThrottled(node, buffer, fromFrame, toFrame, 0.5, throtTimer, transform);
    }

    return evalTransformLookedUp;
}



// bool lookupTransform(rclcpp::Node::SharedPtr node, const std::string& fromFrame, const std::string& toFrame, geometry_msgs::msg::TransformStamped& transform) {
//     bool evalTransformLookedUp = false;
//     tf2_ros::Buffer buffer(node->get_clock());
//     tf2_ros::TransformListener listener(buffer);
//     rclcpp::Time lookupStart = node->get_clock()->now();
//     while(!evalTransformLookedUp && node->get_clock()->now() - lookupStart < 3s) {
//         try {
//             transform = buffer.lookupTransform(toFrame, fromFrame, lookupStart);
//             evalTransformLookedUp = true;
//         } catch(tf2::TransformException& ex) {
//             RCLCPP_WARN_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 500, "Failed to lookup transform from %s to %s (%s)", fromFrame.c_str(), toFrame.c_str(), ex.what());
//         }
//     }

//     return evalTransformLookedUp;
// }


// BT::NodeStatus frameAlignTest(
//     std::shared_ptr<BtTestTool> toolNode,
//     tf2_ros::StaticTransformBroadcaster& broadcaster,
//     const DualVector3& baseLinkOffset,
//     const DualVector3& linkOffset, 
//     const DualVector3& linkGoal,
//     const std::string& linkName,
//     bool& outputsSet,
//     bool publishBaselink = true,
//     bool publishChildLink = true)
// {
//     std::vector<geometry_msgs::msg::TransformStamped> transforms;
//     DualVector3 result;
    
//     if(publishBaselink) {
//         transforms.push_back(createTransform(toolNode, baseLinkOffset.v1, baseLinkOffset.v2, "odom", "bt_testing/base_link"));
//     }

//     if(publishChildLink) {
//         transforms.push_back(createTransform(toolNode, linkOffset.v1, linkOffset.v2, "bt_testing/base_link", linkName));
//     }

//     broadcaster.sendTransform(transforms);

//     //set up ports
//     BT::NodeConfiguration cfg;
//     cfg.input_ports["x"] = std::to_string(linkGoal.v1.x);
//     cfg.input_ports["y"] = std::to_string(linkGoal.v1.y);
//     cfg.input_ports["z"] = std::to_string(linkGoal.v1.z);
//     cfg.input_ports["or"] = std::to_string(linkGoal.v2.x);
//     cfg.input_ports["op"] = std::to_string(linkGoal.v2.y);
//     cfg.input_ports["oy"] = std::to_string(linkGoal.v2.z);
//     cfg.input_ports["link_name"] = linkName;
    
//     //run node
//     auto node = toolNode->createLeafNodeFromConfig("ComputeFrameAlignment", cfg);
//     BT::NodeStatus status = toolNode->tickUntilFinished(node);

//     outputsSet = true;
//     BT::Blackboard::Ptr blackboard = node->config().blackboard;
//     outputsSet = outputsSet && getOutputFromBlackboard<double>(toolNode, node->config().blackboard, "out_x", result.v1.x);
//     outputsSet = outputsSet && getOutputFromBlackboard<double>(toolNode, node->config().blackboard, "out_y", result.v1.y);
//     outputsSet = outputsSet && getOutputFromBlackboard<double>(toolNode, node->config().blackboard, "out_z", result.v1.z);
//     outputsSet = outputsSet && getOutputFromBlackboard<double>(toolNode, node->config().blackboard, "out_or", result.v2.x);
//     outputsSet = outputsSet && getOutputFromBlackboard<double>(toolNode, node->config().blackboard, "out_op", result.v2.y);
//     outputsSet = outputsSet && getOutputFromBlackboard<double>(toolNode, node->config().blackboard, "out_oy", result.v2.z);

//     broadcaster.sendTransform({
//         //publish computed base link position
//         createTransform(toolNode, result.v1, result.v2, "odom", "eval_base_link"),
        
//         //publish preset link offset
//         createTransform(toolNode, linkOffset.v1, linkOffset.v2, "eval_base_link", "eval_link")
//     });

//     #ifdef PRINT_RESULTS_OF_TEST
//         RCLCPP_INFO(toolNode->get_logger(), "bl goal is %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
//             result.v1.x,
//             result.v1.y,
//             result.v1.z,
//             result.v2.x,
//             result.v2.y,
//             result.v2.z);
//     #endif

//     return status;
// }






// TEST_F(BtTest, test_ComputeFrameAlignment_success_exactly_baselink_at_zero_to_zero_with_zero_offset) {
//     tf2_ros::StaticTransformBroadcaster broadcaster(toolNode);
//     bool outputsSet;
//     DualVector3 
//         baseLinkOffset = makeDualVector3(0, 0, 0, 0, 0, 0),
//         childLinkOffset = makeDualVector3(0, 0, 0, 0, 0, 0),
//         linkGoal = makeDualVector3(0, 0, 0, 0, 0, 0);
    
//     BT::NodeStatus status = frameAlignTest(
//         toolNode,
//         broadcaster,
//         baseLinkOffset,
//         childLinkOffset,
//         linkGoal,
//         "test_link",
//         outputsSet
//     );

//     ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
//     ASSERT_TRUE(outputsSet);
//     ASSERT_LINK_NEAR_POS(toolNode, "eval_link", linkGoal.v1, linkGoal.v2, 0.01);
// }

// TEST_F(BtTest, test_ComputeFrameAlignment_success_baselink_at_nonzero_without_rotation_to_zero_with_zero_offset) {
//     tf2_ros::StaticTransformBroadcaster broadcaster(toolNode);
//     bool outputsSet;
//     DualVector3 
//         baseLinkOffset = makeDualVector3(-1, 2, 5, 0, 0, 0),
//         childLinkOffset = makeDualVector3(0, 0, 0, 0, 0, 0),
//         linkGoal = makeDualVector3(0, 0, 0, 0, 0, 0);
    
//     BT::NodeStatus status = frameAlignTest(
//         toolNode,
//         broadcaster,
//         baseLinkOffset,
//         childLinkOffset,
//         linkGoal,
//         "test_link",
//         outputsSet
//     );

//     ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
//     ASSERT_TRUE(outputsSet);
//     ASSERT_LINK_NEAR_POS(toolNode, "eval_link", linkGoal.v1, linkGoal.v2, 0.01);
// }

// TEST_F(BtTest, test_ComputeFrameAlignment_success_baselink_at_nonzero_with_rotation_to_zero_with_zero_offset) {
//     tf2_ros::StaticTransformBroadcaster broadcaster(toolNode);
//     bool outputsSet;
//     DualVector3 
//         baseLinkOffset = makeDualVector3(1, 2, 3, 4, 5, 6), //base link offset
//         childLinkOffset = makeDualVector3(0, 0, 0, 0, 0, 0),
//         linkGoal = makeDualVector3(0, 0, 0, 0, 0, 0);
    
//     BT::NodeStatus status = frameAlignTest(
//         toolNode,
//         broadcaster,
//         baseLinkOffset,
//         childLinkOffset,
//         linkGoal,
//         "test_link",
//         outputsSet
//     );

//     ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
//     ASSERT_TRUE(outputsSet);
//     ASSERT_LINK_NEAR_POS(toolNode, "eval_link", linkGoal.v1, linkGoal.v2, 0.01);
// }

// TEST_F(BtTest, test_ComputeFrameAlignment_success_baselink_at_nonzero_to_nonzero_without_rotation_with_zero_offset) {
//     tf2_ros::StaticTransformBroadcaster broadcaster(toolNode);
//     bool outputsSet;
//     DualVector3 
//         baseLinkOffset = makeDualVector3(1, 2, 3, 4, 5, 6), //base link offset
//         childLinkOffset = makeDualVector3(0, 0, 0, 0, 0, 0),
//         linkGoal = makeDualVector3(4.32, -3.2, -0.4, 0, 0, 0); //link goal
    
//     BT::NodeStatus status = frameAlignTest(
//         toolNode,
//         broadcaster,
//         baseLinkOffset,
//         childLinkOffset,
//         linkGoal,
//         "test_link",
//         outputsSet
//     );

//     ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
//     ASSERT_TRUE(outputsSet);
//     ASSERT_LINK_NEAR_POS(toolNode, "eval_link", linkGoal.v1, linkGoal.v2, 0.01);
// }

// TEST_F(BtTest, test_ComputeFrameAlignment_success_baselink_at_nonzero_to_nonzero_with_rotation_with_zero_offset) {
//     tf2_ros::StaticTransformBroadcaster broadcaster(toolNode);
//     bool outputsSet;
//     DualVector3 
//         baseLinkOffset = makeDualVector3(1, 2, 3, 4, 5, 6), //base link offset
//         childLinkOffset = makeDualVector3(0, 0, 0, 0, 0, 0),
//         linkGoal = makeDualVector3(4.32, -3.2, -0.4, 0, 0, -0.75); //link goal
    
//     BT::NodeStatus status = frameAlignTest(
//         toolNode,
//         broadcaster,
//         baseLinkOffset,
//         childLinkOffset,
//         linkGoal,
//         "test_link",
//         outputsSet
//     );

//     ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
//     ASSERT_TRUE(outputsSet);
//     ASSERT_LINK_NEAR_POS(toolNode, "eval_link", linkGoal.v1, linkGoal.v2, 0.01);
// }

// TEST_F(BtTest, test_ComputeFrameAlignment_success_baselink_at_nonzero_to_nonzero_with_nonzero_offset_without_rotation) {
//     tf2_ros::StaticTransformBroadcaster broadcaster(toolNode);
//     bool outputsSet;
//     DualVector3
//         baseLinkOffset = makeDualVector3(1, 2, 3, 4, 5, 6), 
//         childLinkOffset = makeDualVector3(7.8, -0.2, 1.3, 0, 0, 0), 
//         linkGoal = makeDualVector3(4.32, -3.2, -0.4, 0, 0, -0.75);
    
//     BT::NodeStatus status = frameAlignTest(
//         toolNode,
//         broadcaster,
//         baseLinkOffset,
//         childLinkOffset,
//         linkGoal,
//         "test_link",
//         outputsSet
//     );

//     ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
//     ASSERT_TRUE(outputsSet);
//     ASSERT_LINK_NEAR_POS(toolNode, "eval_link", linkGoal.v1, linkGoal.v2, 0.01);
// }

// TEST_F(BtTest, test_ComputeFrameAlignment_success_baselink_at_nonzero_to_nonzero_with_nonzero_offset_with_rotation) {
//     for(double yaw = 0; yaw < 2 * M_PI; yaw += M_PI / 8) {
//         std::cout << "Setting up test for yaw " << yaw << std::endl;
//         tf2_ros::StaticTransformBroadcaster broadcaster(toolNode);
//         bool outputsSet;
//         DualVector3
//             baseLinkOffset = makeDualVector3(1, 2, 3, 4, 5, 6),
//             childLinkOffset = makeDualVector3(7.8, -0.2, 1.3, 0, 0, 0),
//             linkGoal = makeDualVector3(4.32, -3.2, -0.4, 0, 0, -0.75);
        
//         BT::NodeStatus status = frameAlignTest(
//             toolNode,
//             broadcaster,
//             baseLinkOffset,
//             childLinkOffset,
//             linkGoal,
//             "test_link",
//             outputsSet
//         );
        
//         std::cout << "Eval result of test for yaw " << yaw << std::endl;
//         ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
//         ASSERT_TRUE(outputsSet);
//         ASSERT_LINK_NEAR_POS(toolNode, "eval_link", linkGoal.v1, linkGoal.v2, 0.01);
//     }
// }

// //TODO: add tests for link offset with rotation. right now (at competition) this is not important because the links we are aligning
// //match the base_link's rotation.

// TEST_F(BtTest, test_ComputeFrameAlignment_fail_no_childlink) {
//     tf2_ros::StaticTransformBroadcaster broadcaster(toolNode);
//     bool outputsSet;
//     BT::NodeStatus status = frameAlignTest(
//         toolNode,
//         broadcaster,
//         makeDualVector3(1, 2, 3, 4, 5, 6), //base link offset
//         makeDualVector3(7.8, -0.2, 1.3, 0, 0, 0), //link offset (from base link)
//         makeDualVector3(4.32, -3.2, -0.4, 3, 4.56, -0.75), //link goal
//         "test_link",
//         outputsSet,
//         true,
//         false
//     );

//     ASSERT_EQ(status, BT::NodeStatus::FAILURE);
// }



































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
    ASSERT_LINK_NEAR_POS(toolNode, "eval_link", childLinkGoal, 0.01);
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
    ASSERT_LINK_NEAR_POS(toolNode, "eval_link", childLinkGoal, 0.01);
}

TEST_F(BtTest, test_ComputeFrameAlignment_offset_child_with_rotation_to_zero) {
    tf2_ros::StaticTransformBroadcaster broadcaster(toolNode);
    bool outputsSet;

    DualVector3
        baseLinkPose    = makeDualVector3(0, 0, 0, 0, 0, 0),
        childLinkPose   = makeDualVector3(1, 2, 3, 0, 0, 3.1415),
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
    ASSERT_LINK_NEAR_POS(toolNode, "eval_link", childLinkGoal, 0.01);
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
    ASSERT_LINK_NEAR_POS(toolNode, "eval_link", childLinkGoal, 0.01);
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
    ASSERT_LINK_NEAR_POS(toolNode, "eval_link", childLinkGoal, 0.01);
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
    ASSERT_LINK_NEAR_POS(toolNode, "eval_link", childLinkGoal, 0.01);
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
    ASSERT_LINK_NEAR_POS(toolNode, "eval_link", childLinkGoal, 0.01);
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
    ASSERT_LINK_NEAR_POS(toolNode, "eval_link", childLinkGoal, 0.01);
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
    ASSERT_LINK_NEAR_POS(toolNode, "eval_link", childLinkGoal, 0.01);
}
