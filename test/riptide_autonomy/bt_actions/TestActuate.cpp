#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/BufferedSubscriber.hpp"

using namespace std::chrono_literals;

//NOTE: becuase the actuate node is deprecated in favor of new fancy action client actuators, Ive only defined one test

TEST_F(BtTest, test_Actuate_empty) {
    BT::NodeConfiguration cfg;

    //set up subscription
    BufferedSubscriber<riptide_msgs2::msg::ActuatorCommand> sub(toolNode, ACTUATOR_COMMAND_TOPIC);

    //set up node
    auto node = toolNode->createLeafNodeFromConfig("Actuate", cfg);

    //tick until a message is received
    rclcpp::Time startTime = toolNode->get_clock()->now();
    auto mostRecentStatus = BT::NodeStatus::IDLE;
    while(sub.getMessages().size() < 1 && toolNode->get_clock()->now() - startTime < 2s) {
        mostRecentStatus = node->executeTick();
        toolNode->spinForTime(1ms);
    }

    ASSERT_EQ(mostRecentStatus, BT::NodeStatus::SUCCESS);

    riptide_msgs2::msg::ActuatorCommand expectedCmd; //empty
    ASSERT_EQ(sub.getMessages().size(), (unsigned int) 1);
    ASSERT_EQ(sub.getMessages()[0], expectedCmd);
}

