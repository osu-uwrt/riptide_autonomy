#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/BufferedSubscriber.hpp"

using namespace std::chrono_literals;

TEST(BtTest, test_Actuate_empty) {
    BT::NodeConfiguration cfg;

    //set up subscription
    BufferedSubscriber<riptide_msgs2::msg::ActuatorCommand> sub(BtTestEnvironment::getBtTestTool(), "command/actuator");

    std::cout << "flag -3" << std::endl;

    //set up node
    auto node = BtTestEnvironment::getBtTestTool()->createLeafNodeFromConfig("Actuate", cfg);


    std::cout << "flag -1" << std::endl;

    //tick until a message is received
    rclcpp::Time startTime = BtTestEnvironment::getBtTestTool()->get_clock()->now();
    auto mostRecentStatus = BT::NodeStatus::IDLE;
    while(sub.getMessages().size() < 1 && BtTestEnvironment::getBtTestTool()->get_clock()->now() - startTime < 2s) {
        mostRecentStatus = node->executeTick();

        usleep(10000); //sleep 10000 microseconds
    }

    std::cout << "flag 0" << std::endl;

    ASSERT_EQ(mostRecentStatus, BT::NodeStatus::SUCCESS);

    riptide_msgs2::msg::ActuatorCommand expectedCmd; //empty
    ASSERT_EQ(sub.getMessages().size(), (unsigned int) 1);

    std::cout << "flag 1" << std::endl;
    ASSERT_EQ(sub.getMessages()[0], expectedCmd);
    std::cout << "flag 2" << std::endl;
}

//TODO: define more tests here
