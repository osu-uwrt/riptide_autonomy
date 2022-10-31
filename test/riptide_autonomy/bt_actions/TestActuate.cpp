#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/BufferedSubscriber.hpp"

using namespace std::chrono_literals;

TEST(BtTest, test_Actuate_empty) {
    BT::NodeConfiguration cfg;

    //set up subscription
    BufferedSubscriber<riptide_msgs2::msg::ActuatorCommand> sub(BtTestEnvironment::getBtTestTool(), "command/actuator");

    //set up node
    auto node = BtTestEnvironment::getBtTestTool()->createLeafNodeFromConfig("Actuate", cfg);
    auto result = BtTestEnvironment::getBtTestTool()->tickUntilFinished(node);

    ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
    
    //tick until a message is received
    rclcpp::Time startTime = BtTestEnvironment::getBtTestTool()->get_clock()->now();
    while(sub.getMessages().size() < 1 && BtTestEnvironment::getBtTestTool()->get_clock()->now() - startTime < 2s) {
        BtTestEnvironment::getBtTestTool()->tickUntilFinished(node);
    }

    riptide_msgs2::msg::ActuatorCommand expectedCmd; //empty
    ASSERT_EQ(sub.getMessages().size(), (unsigned int) 1);
    ASSERT_EQ(sub.getMessages()[0], expectedCmd);
}

//TODO: define more tests here
