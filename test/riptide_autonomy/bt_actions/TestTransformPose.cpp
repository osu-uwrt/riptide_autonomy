#include "autonomy_test/autonomy_testing.hpp"

static geometry_msgs::msg::TransformStamped createTransform(double x, double y, double z, double roll, double pitch, double yaw, std::string parentFrame, std::string childFrame) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = rclcpp::Time(0);

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

class TransformPoseTest : public ::testing::Test {
    public:

    static void SetUpTestSuite() {
        //set up static transform broadcasters
        broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(BtTestEnvironment::getBtTestTool());
        
        // std::vector<geometry_msgs::msg::TransformStamped> transforms;
        // transforms.push_back(createTransform(3, 4, -5, 0, 0, 90, "world", "transA"));
        // transforms.push_back(createTransform(-3, 2, -1, 0, 0, -45, "world", "transB"));
        // transforms.push_back(createTransform(-2, -2, -2, 0, 0, 180, "transA", "transB"));

        // broadcaster->sendTransform(transforms);

        broadcaster->sendTransform(createTransform(3, 4, -5, 0, 0, 90, "world", "transA"));     
    }

    static void TearDownTestSuite() { }

    private:
    static std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster;
};


TEST(TransformPoseTest, test_TransformPose_A_to_world) {
    BT::NodeConfiguration cfg;
    cfg.input_ports["from_frame"] = "transA";
    cfg.input_ports["to_frame"] = "world";
    cfg.input_ports["x"] = "1";
    cfg.input_ports["y"] = "1";
    cfg.input_ports["z"] = "1";
    cfg.input_ports["or"] = "0";
    cfg.input_ports["op"] = "0";
    cfg.input_ports["oy"] = "3.1415";
    
    auto node = BtTestEnvironment::getBtTestTool()->createLeafNodeFromConfig("TransformPose", cfg);
    auto result = BtTestEnvironment::getBtTestTool()->tickUntilFinished(node);

    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);

    const double UNDEFINED_VALUE = 999.99;
    auto blackboard = node->config().blackboard;

    double 
        resX = getFromBlackboardWithDefault<double>(blackboard, "out_x", UNDEFINED_VALUE),
        resY = getFromBlackboardWithDefault<double>(blackboard, "out_x", UNDEFINED_VALUE),
        resZ = getFromBlackboardWithDefault<double>(blackboard, "out_x", UNDEFINED_VALUE),
        resRoll = getFromBlackboardWithDefault<double>(blackboard, "out_x", UNDEFINED_VALUE),
        resPitch = getFromBlackboardWithDefault<double>(blackboard, "out_x", UNDEFINED_VALUE),
        resYaw = getFromBlackboardWithDefault<double>(blackboard, "out_x", UNDEFINED_VALUE);

    ASSERT_NEAR(resX, 1, 0.00001);
    ASSERT_NEAR(resY, 1, 0.00001);
    ASSERT_NEAR(resZ, 1, 0.00001);
    ASSERT_NEAR(resRoll, 0, 0.00001);
    ASSERT_NEAR(resPitch, 0, 0.00001);
    ASSERT_NEAR(resYaw, 3.1415, 0.00001);
}

//TODO: define more tests here
