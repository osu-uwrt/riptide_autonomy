#include "autonomy_test/autonomy_testing.hpp"


//
// TEST UTIL THINGS
//

const double FAILED_BB_LOOKUP = 999.99;

static geometry_msgs::msg::TransformStamped createTransform(std::shared_ptr<BtTestTool> toolNode, double x, double y, double z, double roll, double pitch, double yaw, std::string parentFrame, std::string childFrame) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = toolNode->get_clock()->now();

    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = z;
    
    geometry_msgs::msg::Vector3 transformRPY;
    transformRPY.x = roll;
    transformRPY.y = pitch;
    transformRPY.z = yaw;

    transform.transform.rotation = toQuat(transformRPY);

    transform.header.frame_id = parentFrame;
    transform.child_frame_id = childFrame;

    return transform;
}

class FrameAlignTest : public BtTest {
    protected:
    void setUpBroadcaster(
        double baselinkX,
        double baselinkY,
        double baselinkZ,
        double baselinkRoll,
        double baselinkPitch,
        double baselinkYaw)
    {
        //set up static transform broadcasters
        broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(toolNode);
        std::vector<geometry_msgs::msg::TransformStamped> transforms;

        transforms.push_back(
            createTransform(
                toolNode, 
                baselinkX, baselinkY, baselinkZ, 
                baselinkRoll, baselinkPitch, baselinkYaw, 
                "odom", "tempest/base_link"));


        transforms.push_back(createTransform(toolNode, 0, 0, 0, 0, 0, 0, "world", "odom"));
        transforms.push_back(createTransform(toolNode, -0.5, 0.25, 0.125, 0, 0, 0, "tempest/base_link", "some_frame"));
        transforms.push_back(createTransform(toolNode, 0, 0, 0, 0, 0, 0, "tempest/base_link", "base_link_wannabe"));
        transforms.push_back(createTransform(toolNode, -2, -2, -2, 0, 0, 0, "tempest/base_link", "some_other_frame"));

        broadcaster->sendTransform(transforms);
    }

    void TearDown() override {
        broadcaster.reset();
        BtTest::TearDown();
    }

    private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster;
};


BT::NodeStatus testFrameAlign(
    std::shared_ptr<BtTestTool> toolNode, 
    std::string frameName,
    geometry_msgs::msg::Vector3 inPos, 
    geometry_msgs::msg::Vector3& resultPos) 
{
    BT::NodeConfiguration cfg;
    cfg.input_ports["x"] = std::to_string(inPos.x);
    cfg.input_ports["y"] = std::to_string(inPos.y);
    cfg.input_ports["z"] = std::to_string(inPos.z);
    cfg.input_ports["frameName"] = frameName;

    auto node = toolNode->createLeafNodeFromConfig("ComputeFrameAlignment", cfg);
    auto result = toolNode->tickUntilFinished(node);

    auto blackboard = node->config().blackboard;
    resultPos.x = getFromBlackboardWithDefault<double>(blackboard, "out_x", FAILED_BB_LOOKUP);
    resultPos.y = getFromBlackboardWithDefault<double>(blackboard, "out_y", FAILED_BB_LOOKUP);
    resultPos.z = getFromBlackboardWithDefault<double>(blackboard, "out_z", FAILED_BB_LOOKUP);

    return result;
}

TEST_F(FrameAlignTest, test_ComputeFrameAlignment_exactly_baselink) {
    //test computing target position to put some_frame at (-4, -8, -3)
    setUpBroadcaster(1, -1, -1, 0, 0, 0.635);

    geometry_msgs::msg::Vector3 inPos, outPos;
    inPos.x = 2;
    inPos.y = 5;
    inPos.z = -3;

    auto result = testFrameAlign(toolNode, "base_link_wannabe", inPos, outPos);

    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    ASSERT_NEAR(outPos.x, 2, 0.01);
    ASSERT_NEAR(outPos.y, 5, 0.01);
    ASSERT_NEAR(outPos.z, -3, 0.01);
}

TEST_F(FrameAlignTest, test_ComputeFrameAlignment_someframe) {
    //test computing target position to put the frame at (-4, -8, -3)
    setUpBroadcaster(1, -1, -1, 0, 0, 0);

    geometry_msgs::msg::Vector3 inPos, outPos;
    inPos.x = -4;
    inPos.y = -8;
    inPos.z = -3;

    auto result = testFrameAlign(toolNode, "some_frame", inPos, outPos);

    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    ASSERT_NEAR(outPos.x, -3.5, 0.01);
    ASSERT_NEAR(outPos.y, -8.25, 0.01);
    ASSERT_NEAR(outPos.z, -3.125, 0.01);
}

TEST_F(FrameAlignTest, test_ComputeFrameAlignment_someframe_yawed) {
    //test computing target position to put the frame at (-4, -8, -3)
    setUpBroadcaster(1, -1, -1, 0, 0, 1.5707);

    geometry_msgs::msg::Vector3 inPos, outPos;
    inPos.x = -4;
    inPos.y = -8;
    inPos.z = -3;

    auto result = testFrameAlign(toolNode, "some_frame", inPos, outPos);

    //-3.750, -7.500, -3.125
    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    ASSERT_NEAR(outPos.x, -3.75, 0.01);
    ASSERT_NEAR(outPos.y, -7.5, 0.01);
    ASSERT_NEAR(outPos.z, -3.125, 0.01);
}

TEST_F(FrameAlignTest, test_ComputeFrameAlignment_otherframe) {
    //test computing target position to put the frame at (-4, -8, -3)
    setUpBroadcaster(1, -1, -1, 0, 0, 0);

    geometry_msgs::msg::Vector3 inPos, outPos;
    inPos.x = 2;
    inPos.y = 2;
    inPos.z = 2;

    auto result = testFrameAlign(toolNode, "some_other_frame", inPos, outPos);

    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    ASSERT_NEAR(outPos.x, 0, 0.01);
    ASSERT_NEAR(outPos.y, 0, 0.01);
    ASSERT_NEAR(outPos.z, 0, 0.01);
}

TEST_F(FrameAlignTest, test_ComputeFrameAlignment_otherframe_yawed) {
    //test computing target position to put the frame at (-4, -8, -3)
    setUpBroadcaster(1, -1, -1, 0, 0, 1.5707);

    geometry_msgs::msg::Vector3 inPos, outPos;
    inPos.x = 2;
    inPos.y = 2;
    inPos.z = 2;

    auto result = testFrameAlign(toolNode, "some_other_frame", inPos, outPos);

    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    ASSERT_NEAR(outPos.x, 4, 0.01);
    ASSERT_NEAR(outPos.y, 4, 0.01);
    ASSERT_NEAR(outPos.z, 0, 0.01);
}
