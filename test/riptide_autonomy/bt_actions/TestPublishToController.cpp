#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/BufferedSubscriber.hpp"

using namespace std::chrono_literals;

static std::tuple<BT::NodeStatus, std::vector<riptide_msgs2::msg::ControllerCommand> > publishTest(std::shared_ptr<BtTestTool> toolNode, bool orientation, int mode, double x, double y, double z, const std::string expectedTopic) {
    //create sub
    BufferedSubscriber<riptide_msgs2::msg::ControllerCommand> sub(toolNode, expectedTopic);
    
    //set up and create bt node
    BT::NodeConfiguration cfg;
    cfg.input_ports["isOrientation"] = (orientation ? "1" : "0");
    cfg.input_ports["mode"] = std::to_string(mode);
    cfg.input_ports["x"] = std::to_string(x);
    cfg.input_ports["y"] = std::to_string(y);
    cfg.input_ports["z"] = std::to_string(z);

    auto node = toolNode->createLeafNodeFromConfig("PublishToController", cfg);

    //tick for 1 second tops until messages received
    rclcpp::Time startTime = toolNode->get_clock()->now();
    BT::NodeStatus mostRecentStatus = BT::NodeStatus::IDLE;
    while(rclcpp::ok() && sub.getMessages().size() <= 0 && toolNode->get_clock()->now() - startTime < 1s) {
        mostRecentStatus = node->executeTick();

        usleep(10000); //sleep 10000 microseconds
    }

    return std::make_tuple(mostRecentStatus, sub.getMessages());
}

TEST_F(BtTest, test_PublishToController_linear_position) {
    auto result = publishTest (
        toolNode,
        false,
        riptide_msgs2::msg::ControllerCommand::POSITION,
        1.2,
        3.4,
        5.6,
        POSITION_TOPIC
    );

    ASSERT_EQ(std::get<0>(result), BT::NodeStatus::SUCCESS);
    ASSERT_GT(std::get<1>(result).size(), 0U); //this assert MUST happen before below asserts or else segfault may happen
    ASSERT_EQ(std::get<1>(result)[0].mode, riptide_msgs2::msg::ControllerCommand::POSITION);
    ASSERT_NEAR(std::get<1>(result)[0].setpoint_vect.x, 1.2, 0.00001);
    ASSERT_NEAR(std::get<1>(result)[0].setpoint_vect.y, 3.4, 0.00001);
    ASSERT_NEAR(std::get<1>(result)[0].setpoint_vect.z, 5.6, 0.00001);
}

TEST_F(BtTest, test_PublishToController_linear_velocity) {
    auto result = publishTest (
        toolNode,
        false,
        riptide_msgs2::msg::ControllerCommand::VELOCITY,
        0.2,
        7.5,
        2.3,
        POSITION_TOPIC
    );

    ASSERT_EQ(std::get<0>(result), BT::NodeStatus::SUCCESS);
    ASSERT_GT(std::get<1>(result).size(), 0U); //this assert MUST happen before below asserts or else segfault may happen
    ASSERT_EQ(std::get<1>(result)[0].mode, riptide_msgs2::msg::ControllerCommand::VELOCITY);
    ASSERT_NEAR(std::get<1>(result)[0].setpoint_vect.x, 0.2, 0.00001);
    ASSERT_NEAR(std::get<1>(result)[0].setpoint_vect.y, 7.5, 0.00001);
    ASSERT_NEAR(std::get<1>(result)[0].setpoint_vect.z, 2.3, 0.00001);
}

TEST_F(BtTest, test_PublishToController_angular_position) {
    geometry_msgs::msg::Vector3 rpy;
    rpy.x = 62;
    rpy.y = 21;
    rpy.z = 87;

    auto result = publishTest (
        toolNode,
        true,
        riptide_msgs2::msg::ControllerCommand::POSITION,
        rpy.x,
        rpy.y,
        rpy.z,
        ORIENTATION_TOPIC
    );

    ASSERT_EQ(std::get<0>(result), BT::NodeStatus::SUCCESS);
    ASSERT_GT(std::get<1>(result).size(), 0U); //this assert MUST happen before below asserts or else segfault may happen
    
    geometry_msgs::msg::Quaternion expectedQuat = toQuat(rpy);
    ASSERT_EQ(std::get<1>(result)[0].mode, riptide_msgs2::msg::ControllerCommand::POSITION);

    geometry_msgs::msg::Quaternion actualQuat = std::get<1>(result)[0].setpoint_quat;
    ASSERT_NEAR(actualQuat.x, expectedQuat.x, 0.00001);
    ASSERT_NEAR(actualQuat.y, expectedQuat.y, 0.00001);
    ASSERT_NEAR(actualQuat.z, expectedQuat.z, 0.00001);
    ASSERT_NEAR(actualQuat.w, expectedQuat.w, 0.00001);
}

TEST_F(BtTest, test_PublishToController_angular_velocity) {
    auto result = publishTest (
        toolNode,
        true,
        riptide_msgs2::msg::ControllerCommand::VELOCITY,
        3.1415,
        0.1,
        0.5,
        ORIENTATION_TOPIC
    );

    ASSERT_EQ(std::get<0>(result), BT::NodeStatus::SUCCESS);
    ASSERT_GT(std::get<1>(result).size(), 0U); //this assert MUST happen before below asserts or else segfault may happen
    ASSERT_EQ(std::get<1>(result)[0].mode, riptide_msgs2::msg::ControllerCommand::VELOCITY);
    ASSERT_NEAR(std::get<1>(result)[0].setpoint_vect.x, 3.1415, 0.00001);
    ASSERT_NEAR(std::get<1>(result)[0].setpoint_vect.y, 0.1, 0.00001);
    ASSERT_NEAR(std::get<1>(result)[0].setpoint_vect.z, 0.5, 0.00001);
}

TEST_F(BtTest, test_PublishToController_feedforward) {
    auto result = publishTest (
        toolNode,
        false,
        riptide_msgs2::msg::ControllerCommand::FEEDFORWARD,
        0,
        0,
        0,
        POSITION_TOPIC
    );

    ASSERT_EQ(std::get<0>(result), BT::NodeStatus::SUCCESS);
    ASSERT_GT(std::get<1>(result).size(), 0U); //this assert MUST happen before below asserts or else segfault may happen
    ASSERT_EQ(std::get<1>(result)[0].mode, riptide_msgs2::msg::ControllerCommand::FEEDFORWARD);
    ASSERT_NEAR(std::get<1>(result)[0].setpoint_vect.x, 0, 0.00001);
    ASSERT_NEAR(std::get<1>(result)[0].setpoint_vect.y, 0, 0.00001);
    ASSERT_NEAR(std::get<1>(result)[0].setpoint_vect.z, 0, 0.00001);
}

TEST_F(BtTest, test_PublishToController_disabled_linear) {
    auto result = publishTest (
        toolNode,
        false,
        riptide_msgs2::msg::ControllerCommand::DISABLED,
        0,
        0,
        0,
        POSITION_TOPIC
    );

    ASSERT_EQ(std::get<0>(result), BT::NodeStatus::SUCCESS);
    ASSERT_GT(std::get<1>(result).size(), 0U); //this assert MUST happen before below asserts or else segfault may happen
    ASSERT_EQ(std::get<1>(result)[0].mode, riptide_msgs2::msg::ControllerCommand::DISABLED);
    ASSERT_NEAR(std::get<1>(result)[0].setpoint_vect.x, 0, 0.00001);
    ASSERT_NEAR(std::get<1>(result)[0].setpoint_vect.y, 0, 0.00001);
    ASSERT_NEAR(std::get<1>(result)[0].setpoint_vect.z, 0, 0.00001);
}

TEST_F(BtTest, test_PublishToController_disabled_angular) {
    auto result = publishTest (
        toolNode,
        true,
        riptide_msgs2::msg::ControllerCommand::DISABLED,
        0,
        0,
        0,
        ORIENTATION_TOPIC
    );


    ASSERT_EQ(std::get<0>(result), BT::NodeStatus::SUCCESS);
    ASSERT_GT(std::get<1>(result).size(), 0U); //this assert MUST happen before below asserts or else segfault may happen
    
    geometry_msgs::msg::Vector3 resRPY = toRPY(std::get<1>(result)[0].setpoint_quat);

    ASSERT_EQ(std::get<1>(result)[0].mode, riptide_msgs2::msg::ControllerCommand::DISABLED);
    ASSERT_NEAR(resRPY.x, 0, 0.00001);
    ASSERT_NEAR(resRPY.y, 0, 0.00001);
    ASSERT_NEAR(resRPY.z, 0, 0.00001);
}
