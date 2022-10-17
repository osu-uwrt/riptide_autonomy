#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/BufferedSubscriber.hpp"
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

TEST(TestToolTest, test_BufferedSubscriber_noMsgs) {
    BufferedSubscriber<std_msgs::msg::String> bufferedSub(BtTestEnvironment::getBtTestTool(), "test_topic");
    ASSERT_EQ(bufferedSub.getMessages().size(), (unsigned int) 0);

    //sleep 1 second
    usleep(1000000);

    ASSERT_EQ(bufferedSub.getMessages().size(), (unsigned int) 0);
}

TEST(TestToolTest, test_BufferedSubscriber_someMsgs) {
    //the goal of this test is to determine that the buffered sub accurately receives at least 80% of messages thrown at it.
    BufferedSubscriber<std_msgs::msg::String> bufferedSub(BtTestEnvironment::getBtTestTool(), "test_topic");
    auto publisher = BtTestEnvironment::getBtTestTool()->create_publisher<std_msgs::msg::String>("test_topic", 10);

    //publish some messages
    std_msgs::msg::String msg;
    msg.data = "Hello World!";
    for(int i=0; i<10; i++) {
        publisher->publish(msg);
        usleep(50000); //sleep a twentieth of a second
    }

    //how many messages were received?
    unsigned int msgsReceived = numOccurrances<std_msgs::msg::String>(bufferedSub.getMessages(), msg);
    ASSERT_GT(msgsReceived, (unsigned int) 7);
    ASSERT_EQ(msgsReceived, bufferedSub.getMessages().size()); //assert that no messages received incorrectly
}

TEST(TestToolTest, test_BufferedSubscriber_clear) {
    BufferedSubscriber<std_msgs::msg::String> bufferedSub(BtTestEnvironment::getBtTestTool(), "test_topic");
    auto publisher = BtTestEnvironment::getBtTestTool()->create_publisher<std_msgs::msg::String>("test_topic", 10);

    //publish some messages
    std_msgs::msg::String msg;
    msg.data = "Hello World!";
    for(unsigned int i=0; i<10; i++) {
        publisher->publish(msg);
        usleep(100000); //sleep a tenth of a second
    }

    ASSERT_GT(bufferedSub.getMessages().size(), (unsigned int) 7);
    bufferedSub.clearMessages();
    ASSERT_EQ(bufferedSub.getMessages().size(), (unsigned int) 0);
}

TEST(TestToolTest, test_BufferedSubscriber_slowerRate) {
    BufferedSubscriber<std_msgs::msg::String> bufferedSub(BtTestEnvironment::getBtTestTool(), "test_topic");
    auto publisher = BtTestEnvironment::getBtTestTool()->create_publisher<std_msgs::msg::String>("test_topic", 10);

    //publish some messages
    std_msgs::msg::String msg;
    msg.data = "Hello World!";
    for(unsigned int i=0; i<3; i++) {
        publisher->publish(msg);
        usleep(333333); //sleep a third of a second
    }

    ASSERT_GT(bufferedSub.getMessages().size(), (unsigned int) 2);
    bufferedSub.clearMessages();
    ASSERT_EQ(bufferedSub.getMessages().size(), (unsigned int) 0);

    //publish some more messages
    msg.data = "Some Other Message!";
    for(unsigned int i=0; i<3; i++) {
        publisher->publish(msg);
        usleep(333333); //sleep a third of a second
    }

    unsigned int msgsReceived = numOccurrances<std_msgs::msg::String>(bufferedSub.getMessages(), msg);
    ASSERT_GT(msgsReceived, (unsigned int) 2);
}