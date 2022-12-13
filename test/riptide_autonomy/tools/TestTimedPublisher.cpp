#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/TimedPublisher.hpp"
#include <std_msgs/msg/string.hpp>

using namespace std::placeholders;

class TimedPublisherTest : public BtTest {
    protected:
    void init(rclcpp::Node::SharedPtr rosnode, std::string topicName, rclcpp::QoS qos = 10) {
        sub = rosnode->create_subscription<std_msgs::msg::String>(
            topicName, 
            qos, 
            std::bind(&TimedPublisherTest::msgCB, this, _1)
        );

        clock = rosnode->get_clock();
        latestMsgTimestamp = clock->now();
        avgPeriodSeconds = 0;
    }

    std::vector<std_msgs::msg::String> getReceivedMsgs() {
        return receivedMsgs;
    }

    double getAveragePeriod() {
        return avgPeriodSeconds;
    }

    private:
    void msgCB(const std_msgs::msg::String::SharedPtr str) {
        rclcpp::Time currentTime = clock->now();
        double elapsed = (currentTime - latestMsgTimestamp).seconds();

        avgPeriodSeconds *= receivedMsgs.size();
        avgPeriodSeconds += elapsed;
        receivedMsgs.push_back(*str);

        avgPeriodSeconds /= receivedMsgs.size();

        latestMsgTimestamp = currentTime;
    }

    rclcpp::Clock::SharedPtr clock;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
    rclcpp::Time latestMsgTimestamp;
    std::vector<std_msgs::msg::String> receivedMsgs;
    double avgPeriodSeconds;
};


TEST_F(TimedPublisherTest, test_TimedPublisher_qos10_1s) {
    init(toolNode, "test_topic");

    std_msgs::msg::String msg;
    msg.data = "some message...";
    TimedPublisher<std_msgs::msg::String> timedPub(toolNode, "test_topic", msg);

    while(getReceivedMsgs().size() < 3) {
        usleep(10000);
    }

    unsigned int correctMsgs = numOccurrances<std_msgs::msg::String>(getReceivedMsgs(), msg);

    //check that we received between 1 and 5 messages and all of them are correct
    ASSERT_EQ(correctMsgs, (unsigned int) 3);
    ASSERT_EQ(correctMsgs, getReceivedMsgs().size());
    ASSERT_NEAR(getAveragePeriod(), 1, 0.01);
}

TEST_F(TimedPublisherTest, test_TimedPublisher_sensordataqos_quartersecond) {
    init(toolNode, "another_test_topic", rclcpp::SensorDataQoS());

    std_msgs::msg::String msg;
    msg.data = "some message...";
    TimedPublisher<std_msgs::msg::String> timedPub(toolNode, "another_test_topic", msg, rclcpp::SensorDataQoS(), 250);

    while(getReceivedMsgs().size() < 5) {
        usleep(10000);
    }

    unsigned int correctMsgs = numOccurrances<std_msgs::msg::String>(getReceivedMsgs(), msg);

    //check that we received between 6 and 7 messages and all of them are correct
    ASSERT_EQ(correctMsgs, (unsigned int) 5); 
    ASSERT_EQ(correctMsgs, getReceivedMsgs().size());
    ASSERT_NEAR(getAveragePeriod(), 0.25, 0.01);
}

TEST_F(TimedPublisherTest, test_TimedPublisher_changeMsg) {
    init(toolNode, "some_test_topic", rclcpp::SensorDataQoS());

    std_msgs::msg::String msg;
    msg.data = "some message...";
    TimedPublisher<std_msgs::msg::String> timedPub(toolNode, "some_test_topic", msg, rclcpp::SensorDataQoS(), 125);

    //wait for timed pub to publish one message
    while(getReceivedMsgs().size() < 1) {
        usleep(10000); //sleep 1/100 seconds
    }

    ASSERT_EQ(getReceivedMsgs()[0].data, msg.data);
    ASSERT_LT(getReceivedMsgs().size(), (unsigned int) 3); //this is for integrity of the next part of the test

    //change message
    std_msgs::msg::String msg2;
    msg2.data = "another message because testing";
    timedPub.changeMessage(msg2);

    //wait for timed pub to publish one message
    while(getReceivedMsgs().size() < 3) {
        usleep(10000); //sleep 1/100 seconds
    }

    //make sure that 
    ASSERT_EQ(getReceivedMsgs()[2].data, msg2.data);
}

TEST_F(TimedPublisherTest, test_TimedPublsher_cancel) {
    init(toolNode, "maybe_the_last_test_topic", rclcpp::SensorDataQoS());

    std_msgs::msg::String msg;
    msg.data = "some message...";
    TimedPublisher<std_msgs::msg::String> timedPub(toolNode, "maybe_the_last_test_topic", msg, rclcpp::SensorDataQoS(), 125);

    while(getReceivedMsgs().size() < 2) {
        usleep(10000);
    }

    ASSERT_EQ(getReceivedMsgs()[1].data, msg.data);
    timedPub.cancelTimer(); //destruct, should cause it to stop publishing

    unsigned int expectedMsgCount = getReceivedMsgs().size();

    //sleep 1 second. should receive around 8 messages in this time
    usleep(1000000);

    ASSERT_EQ(getReceivedMsgs().size(), expectedMsgCount);
}
