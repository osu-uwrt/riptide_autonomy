#pragma once
#include "autonomy_test/autonomy_testing.hpp"

using namespace std::chrono_literals;

/**
 * @brief A basic implementation for a ROS publisher that publishes on a timed interval.
 * 
 * @tparam T The type of message to publish
 */
template<typename T>
class TimedPublisher {
    public:
    /**
     * @brief Construct a new Timed Publisher object
     * 
     * @param node The ROS node to construct the publisher with.
     * @param topic The topic to register the publisher for
     * @param message The message to publish.
     * @param qos The QoS to use.
     * @param intervalMS The interval in milliseconds.
     */
    TimedPublisher(rclcpp::Node::SharedPtr node, std::string topic, T message, rclcpp::QoS qos = 10, int intervalMS = 1000) {
        this->message = message;

        pub = node->create_publisher<T>(topic, qos);
        timer = node->create_wall_timer(
            std::chrono::milliseconds(intervalMS),
            std::bind(&TimedPublisher::onTimer, this)
        );
    }

    /**
     * @brief Destroy the Timed Publisher object. Will stop the timer
     */
    ~TimedPublisher() {
        timer->cancel();
    }

    /**
     * @brief Changes the message published by the timed publisher.
     * 
     * @param newMessage The new message to publish.
     */
    void changeMessage(T newMessage) {
        message = newMessage;
    }

    private:
    void onTimer() {
        RCLCPP_INFO(log, "Publishing a Message");
        pub->publish(message);
        
    }

    T message;
    rclcpp::TimerBase::SharedPtr timer;
    std::shared_ptr<rclcpp::Publisher<T>> pub;
};
