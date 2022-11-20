#include "autonomy_test/autonomy_testing.hpp"

using namespace std::chrono_literals;

//
// TEST UTIL THINGS
//

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

class TransformPoseTest : public BtTest {
    protected:
    void SetUp() override {
        BtTest::SetUp();

        //set up static transform broadcasters
        broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(toolNode);
        
        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        transforms.push_back(createTransform(toolNode, 3, 4, -5, 0, 0, 1.5707, "world", "transA"));
        transforms.push_back(createTransform(toolNode, -3, 2, -1, 0, 0, -0.78535, "world", "transB"));
        transforms.push_back(createTransform(toolNode, -2, -2, -2, 0, 0, 0, "transA", "transC"));

        broadcaster->sendTransform(transforms);
    }

    void TearDown() override {
        broadcaster.reset();
        BtTest::TearDown();
    }

    private:
    static std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster;
};

std::shared_ptr<tf2_ros::StaticTransformBroadcaster> TransformPoseTest::broadcaster = nullptr; //undefined references are very annoying (var will be un-nullified when test suite set up)


//
// TEST FUNCTION
//

/**
 * @brief Runs the transformPose node on the given inputs and populates the results array with the results.
 * 
 * @param x The x coordinate to transform
 * @param y The y coordinate to transform
 * @param z The z coordinate to transform
 * @param roll The roll to transform
 * @param pitch The pitch to transform
 * @param yaw The yaw to transform
 * @param fromFrame The from to transform from
 * @param toFrame The frame to transform to
 * @param results An array containing the results. Details shown below:
 * [0] - transformed X
 * [1] - transformed Y
 * [2] - transformed Z
 * [3] - transformed roll
 * [4] - transformed pitch
 * [5] - transformed yaw
 * 
 * @return The resulting node status
 */
BT::NodeStatus testTransform(std::shared_ptr<BtTestTool> toolNode, double x, double y, double z, double roll, double pitch, double yaw, std::string fromFrame, std::string toFrame, double results[6]) {
    BT::NodeConfiguration cfg;
    cfg.input_ports["from_frame"] = fromFrame;
    cfg.input_ports["to_frame"] = toFrame;
    cfg.input_ports["x"] = std::to_string(x);
    cfg.input_ports["y"] = std::to_string(y);
    cfg.input_ports["z"] = std::to_string(z);
    cfg.input_ports["or"] = std::to_string(roll);
    cfg.input_ports["op"] = std::to_string(pitch);
    cfg.input_ports["oy"] = std::to_string(yaw);
    
    auto node = toolNode->createLeafNodeFromConfig("TransformPose", cfg);
    auto result = toolNode->tickUntilFinished(node);

    const double UNDEFINED_VALUE = 999.99;
    auto blackboard = node->config().blackboard;

    results[0] = getFromBlackboardWithDefault<double>(blackboard, "out_x", UNDEFINED_VALUE),
    results[1] = getFromBlackboardWithDefault<double>(blackboard, "out_y", UNDEFINED_VALUE),
    results[2] = getFromBlackboardWithDefault<double>(blackboard, "out_z", UNDEFINED_VALUE),
    results[3] = getFromBlackboardWithDefault<double>(blackboard, "out_or", UNDEFINED_VALUE),
    results[4] = getFromBlackboardWithDefault<double>(blackboard, "out_op", UNDEFINED_VALUE),
    results[5] = getFromBlackboardWithDefault<double>(blackboard, "out_oy", UNDEFINED_VALUE);

    return result;
}


//
// TEST CASES
//

TEST_F(TransformPoseTest, test_TransformPose_A_to_world) {
    double results[6];
    auto result = testTransform(toolNode, 1, 1, 1, 0, 0, 1.5707, "transA", "world", results);

    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    ASSERT_NEAR(results[0], 2, 0.001);
    ASSERT_NEAR(results[1], 5, 0.001);
    ASSERT_NEAR(results[2], -4, 0.001);
    ASSERT_NEAR(results[3], 0, 0.001);
    ASSERT_NEAR(results[4], 0, 0.001);
    ASSERT_NEAR(results[5], 3.1415, 0.001);
}

TEST_F(TransformPoseTest, test_TransformPose_B_to_world) {
    double results[6];
    auto result = testTransform(toolNode, 0, 0, 0, 0, 0, 0, "transB", "world", results);

    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    ASSERT_NEAR(results[0], -3, 0.001);
    ASSERT_NEAR(results[1], 2, 0.001);
    ASSERT_NEAR(results[2], -1, 0.001);
    ASSERT_NEAR(results[3], 0, 0.001);
    ASSERT_NEAR(results[4], 0, 0.001);
    ASSERT_NEAR(results[5], -0.78535, 0.001);
}

TEST_F(TransformPoseTest, test_TransformPose_C_to_world) {
    double results[6];
    auto result = testTransform(toolNode, 0, 0, 0, 0, 0, 1.5707, "transC", "world", results);

    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    ASSERT_NEAR(results[0], 5, 0.001);
    ASSERT_NEAR(results[1], 2, 0.001);
    ASSERT_NEAR(results[2], -7, 0.001);
    ASSERT_NEAR(results[3], 0, 0.001);
    ASSERT_NEAR(results[4], 0, 0.001);
    ASSERT_NEAR(results[5], 3.1415, 0.001);
}

TEST_F(TransformPoseTest, test_TransformPose_C_to_A) {
    double results[6];
    auto result = testTransform(toolNode, 2, 2, 2, 0, 0, 0, "transC", "transA", results);

    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    ASSERT_NEAR(results[0], 0, 0.001);
    ASSERT_NEAR(results[1], 0, 0.001);
    ASSERT_NEAR(results[2], 0, 0.001);
    ASSERT_NEAR(results[3], 0, 0.001);
    ASSERT_NEAR(results[4], 0, 0.001);
    ASSERT_NEAR(results[5], 0, 0.001);
}
