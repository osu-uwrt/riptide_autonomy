#pragma once
#include "autonomy_test/autonomy_testing.hpp"

using namespace std::placeholders;

/**
 * @brief A simple class that creates a subscription to a topic and collects all messages gather on that topic to a vector.
 * This was created to streamline testing for behavior tree nodes that publish messages, so that the programmer does not need
 * to manually create subscriptions that collect messages.
 * 
 * @tparam T The type of message to collect
 */
template<typename T> 
class BufferedSubscriber {
    public:
    /**
     * @brief Construct a new Message Aggregator object
     * 
     * @param node The ROS node to use to create the subscription
     * @param topic The topic to subscribe to
     * @param qos The ROS QoS to use on the subscription
     */
    BufferedSubscriber(rclcpp::Node::SharedPtr node, const std::string topic, const rclcpp::QoS qos = 10) {
        sub = node->create_subscription<T>(
            topic,
            qos,
            std::bind(&BufferedSubscriber::msgCallback, this, _1)
        );
    }

    /**
     * @brief Get the messages collected on the topic
     * 
     * @return std::vector<T> containing all messages collected on the topic since creation or the last call to clearMessages()
     */
    std::vector<T> getMessages() {
        return messages;
    }

    /**
     * @brief Clear the vector containing the collected messages
     * 
     */
    void clearMessages() {
        messages.clear();
    }

    private:
    void msgCallback(const std::shared_ptr<T> msg) {
        messages.push_back(*msg);
    }

    std::vector<T> messages;
    std::shared_ptr<rclcpp::Subscription<T>> sub;
};
