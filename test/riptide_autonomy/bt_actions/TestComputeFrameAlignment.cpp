#include "autonomy_test/autonomy_testing.hpp"


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

class FrameAlignTest : public BtTest {
    protected:
    void SetUp() override {
        BtTest::SetUp();

        //set up static transform broadcasters
        broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(toolNode);
        
        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        transforms.push_back(createTransform(toolNode, 3, 4, -5, 0, 0, 1.5707, "world", "tempest/base_link"));
        transforms.push_back(createTransform(toolNode, -3, 2, -1, 0, 0, -0.78535, "tempest/base_link", "some_frame"));
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


TEST_F(FrameAlignTest, test_ComputeFrameAlignment) {
    //TODO: define a test here and delete below line when you do
    GTEST_SKIP();
}

//TODO: define more tests here
