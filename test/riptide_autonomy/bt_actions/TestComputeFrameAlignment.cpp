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


#define ASSERT_LINK_NEAR_POS(node, linkName, xyz, rpy, absError) \
    do { \
        geometry_msgs::msg::TransformStamped transform; \
        bool evalTransformLookedUp = lookupTransform(node, linkName, "odom", transform); \
        ASSERT_TRUE(evalTransformLookedUp); \
        ASSERT_NEAR(transform.transform.translation.x, xyz.x, absError); \
        ASSERT_NEAR(transform.transform.translation.y, xyz.y, absError); \
        ASSERT_NEAR(transform.transform.translation.z, xyz.z, absError); \
        geometry_msgs::msg::Vector3 transRpy = toRPY(transform.transform.rotation); \
        ASSERT_NEAR(transRpy.x, rpy.x, absError); \
        ASSERT_NEAR(transRpy.y, rpy.y, absError); \
        ASSERT_NEAR(transRpy.z, rpy.z, absError); \
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


geometry_msgs::msg::TransformStamped createTransform(std::shared_ptr<BtTestTool> toolNode, geometry_msgs::msg::Vector3 xyz, geometry_msgs::msg::Vector3 rpy, std::string parentFrame, std::string childFrame) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = toolNode->get_clock()->now();

    transform.transform.translation.x = xyz.x;
    transform.transform.translation.y = xyz.y;
    transform.transform.translation.z = xyz.z;
    
    geometry_msgs::msg::Vector3 transformRPY;
    transformRPY.x = rpy.x;
    transformRPY.y = rpy.y;
    transformRPY.z = rpy.z;

    transform.transform.rotation = toQuat(transformRPY);

    transform.header.frame_id = parentFrame;
    transform.child_frame_id = childFrame;

    return transform;
}


bool lookupTransform(rclcpp::Node::SharedPtr node, const std::string& fromFrame, const std::string& toFrame, geometry_msgs::msg::TransformStamped& transform) {
    bool evalTransformLookedUp = false;
    tf2_ros::Buffer buffer(node->get_clock());
    tf2_ros::TransformListener listener(buffer);
    rclcpp::Time lookupStart = node->get_clock()->now();
    while(!evalTransformLookedUp && node->get_clock()->now() - lookupStart < 3s) {
        try {
            transform = buffer.lookupTransform(toFrame, fromFrame, lookupStart);
            evalTransformLookedUp = true;
        } catch(tf2::TransformException& ex) {
            RCLCPP_WARN_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 500, "Failed to lookup transform from %s to %s (%s)", fromFrame.c_str(), toFrame.c_str(), ex.what());
        }
    }

    return evalTransformLookedUp;
}


BT::NodeStatus frameAlignTest(
    std::shared_ptr<BtTestTool> toolNode,
    tf2_ros::StaticTransformBroadcaster& broadcaster,
    const DualVector3& baseLinkOffset,
    const DualVector3& linkOffset, 
    const DualVector3& linkGoal,
    const std::string& linkName,
    bool& outputsSet,
    bool publishBaselink = true,
    bool publishChildLink = true)
{
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    DualVector3 result;
    
    if(publishBaselink) {
        transforms.push_back(createTransform(toolNode, baseLinkOffset.v1, baseLinkOffset.v2, "odom", "bt_testing/base_link"));
    }

    if(publishChildLink) {
        transforms.push_back(createTransform(toolNode, linkOffset.v1, linkOffset.v2, "bt_testing/base_link", linkName));
    }

    broadcaster.sendTransform(transforms);

    //set up ports
    BT::NodeConfiguration cfg;
    cfg.input_ports["x"] = std::to_string(linkGoal.v1.x);
    cfg.input_ports["y"] = std::to_string(linkGoal.v1.y);
    cfg.input_ports["z"] = std::to_string(linkGoal.v1.z);
    cfg.input_ports["or"] = std::to_string(linkGoal.v2.x);
    cfg.input_ports["op"] = std::to_string(linkGoal.v2.y);
    cfg.input_ports["oy"] = std::to_string(linkGoal.v2.z);
    cfg.input_ports["link_name"] = linkName;
    
    //run node
    auto node = toolNode->createLeafNodeFromConfig("ComputeFrameAlignment", cfg);
    BT::NodeStatus status = toolNode->tickUntilFinished(node);

    //set outputs
    // UwrtBtNode *uwrtNode = dynamic_cast<UwrtBtNode *>(node.get());

    outputsSet = true;
    BT::Blackboard::Ptr blackboard = node->config().blackboard;
    outputsSet = outputsSet && getOutputFromBlackboard<double>(toolNode, node->config().blackboard, "out_x", result.v1.x);
    outputsSet = outputsSet && getOutputFromBlackboard<double>(toolNode, node->config().blackboard, "out_y", result.v1.y);
    outputsSet = outputsSet && getOutputFromBlackboard<double>(toolNode, node->config().blackboard, "out_z", result.v1.z);
    outputsSet = outputsSet && getOutputFromBlackboard<double>(toolNode, node->config().blackboard, "out_or", result.v2.x);
    outputsSet = outputsSet && getOutputFromBlackboard<double>(toolNode, node->config().blackboard, "out_op", result.v2.y);
    outputsSet = outputsSet && getOutputFromBlackboard<double>(toolNode, node->config().blackboard, "out_oy", result.v2.z);

    broadcaster.sendTransform({
        //publish computed base link position
        createTransform(toolNode, result.v1, result.v2, "odom", "eval_base_link"),
        
        //publish preset link offset
        createTransform(toolNode, linkOffset.v1, linkOffset.v2, "eval_base_link", "eval_link")
    });

    #ifdef PRINT_RESULTS_OF_TEST
        RCLCPP_INFO(toolNode->get_logger(), "bl goal is %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
            result.v1.x,
            result.v1.y,
            result.v1.z,
            result.v2.x,
            result.v2.y,
            result.v2.z);
    #endif

    return status;
}






// TEST_F(BtTest, test_ComputeFrameAlignment_success_exactly_baselink_at_zero_to_zero_with_zero_offset) {
//     tf2_ros::StaticTransformBroadcaster broadcaster(toolNode);
//     bool outputsSet;
//     DualVector3 result;
//     BT::NodeStatus status = frameAlignTest(
//         toolNode,
//         broadcaster,
//         makeDualVector3(0, 0, 0, 0, 0, 0), //base link offset
//         makeDualVector3(0, 0, 0, 0, 0, 0), //link offset (from base link)
//         makeDualVector3(0, 0, 0, 0, 0, 0), //link goal
//         "test_link",
//         outputsSet,
//         result
//     );

//     ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
//     ASSERT_TRUE(outputsSet);
//     ASSERT_LINK_NEAR_POS(toolNode, "eval_link", result.v1, result.v2, 0.01);
// }

TEST_F(BtTest, test_ComputeFrameAlignment_success_baselink_at_nonzero_without_rotation_to_zero_with_zero_offset) {
    tf2_ros::StaticTransformBroadcaster broadcaster(toolNode);
    bool outputsSet;
    DualVector3 
        baseLinkOffset = makeDualVector3(-1, 2, 5, 0, 0, 0),
        childLinkOffset = makeDualVector3(0, 0, 0, 0, 0, 0),
        linkGoal = makeDualVector3(0, 0, 0, 0, 0, 0);
    
    BT::NodeStatus status = frameAlignTest(
        toolNode,
        broadcaster,
        baseLinkOffset,
        childLinkOffset,
        linkGoal,
        "test_link",
        outputsSet
    );

    ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(outputsSet);
    ASSERT_LINK_NEAR_POS(toolNode, "eval_link", linkGoal.v1, linkGoal.v2, 0.01);
}

// TEST_F(BtTest, test_ComputeFrameAlignment_success_baselink_at_nonzero_with_rotation_to_zero_with_zero_offset) {
//     tf2_ros::StaticTransformBroadcaster broadcaster(toolNode);
//     bool outputsSet;
//     DualVector3 result;
//     BT::NodeStatus status = frameAlignTest(
//         toolNode,
//         broadcaster,
//         makeDualVector3(1, 2, 3, 4, 5, 6), //base link offset
//         makeDualVector3(0, 0, 0, 0, 0, 0), //link offset (from base link)
//         makeDualVector3(0, 0, 0, 0, 0, 0), //link goal
//         "test_link",
//         outputsSet,
//         result
//     );

//     ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
//     ASSERT_TRUE(outputsSet);
//     ASSERT_LINK_NEAR_POS(toolNode, "eval_link", result.v1, result.v2, 0.01);
// }

// TEST_F(BtTest, test_ComputeFrameAlignment_success_baselink_at_nonzero_to_nonzero_without_rotation_with_zero_offset) {
//     tf2_ros::StaticTransformBroadcaster broadcaster(toolNode);
//     bool outputsSet;
//     DualVector3 result;
//     BT::NodeStatus status = frameAlignTest(
//         toolNode,
//         broadcaster,
//         makeDualVector3(1, 2, 3, 4, 5, 6), //base link offset
//         makeDualVector3(0, 0, 0, 0, 0, 0), //link offset (from base link)
//         makeDualVector3(4.32, -3.2, -0.4, 0, 0, 0), //link goal
//         "test_link",
//         outputsSet,
//         result
//     );

//     ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
//     ASSERT_TRUE(outputsSet);
//     ASSERT_LINK_NEAR_POS(toolNode, "eval_link", result.v1, result.v2, 0.01);
// }

// TEST_F(BtTest, test_ComputeFrameAlignment_success_baselink_at_nonzero_to_nonzero_with_rotation_with_zero_offset) {
//     tf2_ros::StaticTransformBroadcaster broadcaster(toolNode);
//     bool outputsSet;
//     DualVector3 result;
//     BT::NodeStatus status = frameAlignTest(
//         toolNode,
//         broadcaster,
//         makeDualVector3(1, 2, 3, 4, 5, 6), //base link offset
//         makeDualVector3(0, 0, 0, 0, 0, 0), //link offset (from base link)
//         makeDualVector3(4.32, -3.2, -0.4, 3, 4.56, -0.75), //link goal
//         "test_link",
//         outputsSet,
//         result
//     );

//     ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
//     ASSERT_TRUE(outputsSet);
//     ASSERT_LINK_NEAR_POS(toolNode, "eval_link", result.v1, result.v2, 0.01);
// }

TEST_F(BtTest, test_ComputeFrameAlignment_success_baselink_at_nonzero_to_nonzero_with_nonzero_offset_without_rotation) {
    tf2_ros::StaticTransformBroadcaster broadcaster(toolNode);
    bool outputsSet;
    DualVector3
        baseLinkOffset = makeDualVector3(1, 2, 3, 4, 5, 6), 
        childLinkOffset = makeDualVector3(7.8, -0.2, 1.3, 0, 0, 0), 
        linkGoal = makeDualVector3(4.32, -3.2, -0.4, 3, 4.56, -0.75);
    
    BT::NodeStatus status = frameAlignTest(
        toolNode,
        broadcaster,
        baseLinkOffset,
        childLinkOffset,
        linkGoal,
        "test_link",
        outputsSet
    );

    ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(outputsSet);
    ASSERT_LINK_NEAR_POS(toolNode, "eval_link", linkGoal.v1, linkGoal.v2, 0.01);
}

TEST_F(BtTest, test_ComputeFrameAlignment_success_baselink_at_nonzero_to_nonzero_with_nonzero_offset_with_rotation) {
    tf2_ros::StaticTransformBroadcaster broadcaster(toolNode);
    bool outputsSet;
    DualVector3
        baseLinkOffset = makeDualVector3(1, 2, 3, 4, 5, 6),
        childLinkOffset = makeDualVector3(7.8, -0.2, 1.3, 0.3, -5.6, 1.23),
        linkGoal = makeDualVector3(4.32, -3.2, -0.4, 3, 4.56, -0.75);
    
    BT::NodeStatus status = frameAlignTest(
        toolNode,
        broadcaster,
        baseLinkOffset,
        childLinkOffset,
        linkGoal,
        "test_link",
        outputsSet
    );

    ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(outputsSet);
    ASSERT_LINK_NEAR_POS(toolNode, "eval_link", linkGoal.v1, linkGoal.v2, 0.01);
}

// TEST_F(BtTest, test_ComputeFrameAlignment_success_misc_1) {
//     tf2_ros::StaticTransformBroadcaster broadcaster(toolNode);
//     bool outputsSet;
//     DualVector3 result;
//     BT::NodeStatus status = frameAlignTest(
//         toolNode,
//         broadcaster,
//         makeDualVector3(1.21, 1.73, 3.9, -5.95, 8.07, 8.86), //base link offset
//         makeDualVector3(9.98, -3.5, 3.05, 6.48, -7.49, -0.18), //link offset (from base link)
//         makeDualVector3(-2.39, -6.45, -9.28, -0.93, 5.48, -0.34), //link goal
//         "test_link",
//         outputsSet,
//         result
//     );

//     ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
//     ASSERT_TRUE(outputsSet);
//     ASSERT_LINK_NEAR_POS(toolNode, "eval_link", result.v1, result.v2, 0.01);
// }

// TEST_F(BtTest, test_ComputeFrameAlignment_success_misc_2_different_frame) {
//     tf2_ros::StaticTransformBroadcaster broadcaster(toolNode);
//     bool outputsSet;
//     DualVector3 result;
//     BT::NodeStatus status = frameAlignTest(
//         toolNode,
//         broadcaster,
//         makeDualVector3(3.2, 0.66, -0.85, -7.97, -6.53, -6.03), //base link offset
//         makeDualVector3(-1.81, -9.99, 2.36, -9.63, -6.55, 9.83), //link offset (from base link)
//         makeDualVector3(8.55, 5.85, 7.7, 1.26, -5.9, -6.37), //link goal
//         "different_link",
//         outputsSet,
//         result
//     );

//     ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
//     ASSERT_TRUE(outputsSet);
//     ASSERT_LINK_NEAR_POS(toolNode, "eval_link", result.v1, result.v2, 0.01);
// }

// TEST_F(BtTest, test_ComputeFrameAlignment_fail_no_baselink) {
//     tf2_ros::StaticTransformBroadcaster broadcaster(toolNode);
//     bool outputsSet;
//     DualVector3 result;
//     BT::NodeStatus status = frameAlignTest(
//         toolNode,
//         broadcaster,
//         makeDualVector3(1, 2, 3, 4, 5, 6), //base link offset
//         makeDualVector3(7.8, -0.2, 1.3, 0, 0, 0), //link offset (from base link)
//         makeDualVector3(4.32, -3.2, -0.4, 3, 4.56, -0.75), //link goal
//         "test_link",
//         outputsSet,
//         result,
//         "eval_link",
//         false,
//         true
//     );

//     ASSERT_EQ(status, BT::NodeStatus::FAILURE);
// }

// TEST_F(BtTest, test_ComputeFrameAlignment_fail_no_childlink) {
//     tf2_ros::StaticTransformBroadcaster broadcaster(toolNode);
//     bool outputsSet;
//     DualVector3 result;
//     BT::NodeStatus status = frameAlignTest(
//         toolNode,
//         broadcaster,
//         makeDualVector3(1, 2, 3, 4, 5, 6), //base link offset
//         makeDualVector3(7.8, -0.2, 1.3, 0, 0, 0), //link offset (from base link)
//         makeDualVector3(4.32, -3.2, -0.4, 3, 4.56, -0.75), //link goal
//         "test_link",
//         outputsSet,
//         result,
//         "eval_link",
//         true,
//         false
//     );

//     ASSERT_EQ(status, BT::NodeStatus::FAILURE);
// }
