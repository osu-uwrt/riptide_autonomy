#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/TimedPublisher.hpp"
#include <std_msgs/msg/string.hpp>

TEST(TestToolTest, test_TimedPublisher_slowInterval) {
    //message to send/receive
    std_msgs::msg::String msg;
    msg.data = "some string";

    int
        numMsgsReceived = 0,
        numMsgsCorrect = 0;
    
    double avgTimePeriod = 0;
    rclcpp::Time mostRecentTimestamp = BtTestEnvironment::getBtTestTool()->get_clock()->now();

    auto sub = BtTestEnvironment::getBtTestTool()->create_subscription<std_msgs::msg::String>(
        "test_topic",
        10,

        //this lambda callback increments msgsReceived if the received message equals msg
        [&numMsgsReceived, &numMsgsCorrect, &avgTimePeriod, &msg, &mostRecentTimestamp] (const std_msgs::msg::String::SharedPtr received) {
            rclcpp::Time currentTime = BtTestEnvironment::getBtTestTool()->get_clock()->now();
            double secondsElapsed = (currentTime - mostRecentTimestamp).seconds();

            avgTimePeriod = ((avgTimePeriod * numMsgsReceived) + secondsElapsed) / (numMsgsReceived + 1);

            if(received->data == msg.data) {
                numMsgsCorrect++;
            }

            numMsgsReceived++;
            mostRecentTimestamp = currentTime;
        }
    );

    //set up timed pub
    TimedPublisher<std_msgs::msg::String> timedPub(BtTestEnvironment::getBtTestTool(), "test_topic", msg); //qos 10 and interval 1 second

    //wait 5 seconds
    usleep(1000000);

    ASSERT_NEAR(avgTimePeriod, 1, 0.05);
    ASSERT_EQ(numMsgsReceived, numMsgsCorrect);
    ASSERT_GT(numMsgsReceived, 3);
}

//TODO: define more tests here
