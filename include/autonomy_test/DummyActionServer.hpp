#pragma once
#include "autonomy_test/autonomy_testing.hpp"

using namespace std::placeholders;

/**
 * @brief A simple server to test action clients on
 * @tparam ActionT The action type to make the server for.
 */
template<typename ActionT>
class DummyActionServer {
    public:
    typedef rclcpp_action::ServerGoalHandle<ActionT> GoalHandleT;
    typedef typename ActionT::Goal ActionGoalT;
    typedef typename ActionT::Feedback ActionFeedbackT;
    typedef typename ActionT::Result ActionResultT;

    /**
     * @brief Creates a new DummyActionServer.
     * @param n The ROS node to attach the server to.
     * @param name The name of the server
     */
    DummyActionServer(rclcpp::Node::SharedPtr n, const std::string& name) {
        this->node = n;
        this->server = rclcpp_action::create_server<ActionT>(
            n, 
            name,
            std::bind(&DummyActionServer::handleGoal, this, _1, _2),
            std::bind(&DummyActionServer::handleCancel, this, _1),
            std::bind(&DummyActionServer::handleAccepted, this, _1)
        );

        this->feedbackEnabled = false;
        this->gotRequest = false;
        this->gotCancel = false;
    }

    /**
     * @brief Configures the server to run without feedback.
     * @param goalResponse The response to new goal requests.
     * @param cancelResponse The response to new cancel requests.
     * @param execTime The duration that execution will take
     * @param succeed true if the server should succeed, false if it should abort.
     * @param result The result to return at the end of the action.
     * @param canceledResult The result to return if the goal is canceled.
     */
    void configureNoFeedback(
        rclcpp_action::GoalResponse goalResponse,
        rclcpp_action::CancelResponse cancelResponse,
        std::chrono::duration<double> execTime,
        bool succeed,
        std::shared_ptr<ActionResultT> result,
        std::shared_ptr<ActionResultT> canceledResult = std::make_shared<ActionResultT>())
    {
        this->goalResponse = goalResponse;
        this->cancelResponse = cancelResponse;
        this->execTime = execTime;
        this->succeed = succeed;
        this->result = result;
        this->canceledResult = canceledResult;
        this->gotRequest = false;
        this->gotCancel = false;
        this->stopped = false;
    }

    /**
     * @brief Configures the server to run with feedback
     * @param goalResponse The response to new goal requests.
     * @param cancelResponse The response to new cancel requests.
     * @param execTime The duration that execution will take
     * @param succeed true if the server should succeed, false if it should abort.
     * @param result The result to return at the end of the action.
     * @param feedback The feedback to send periodically.
     * @param feedbackPeriod The time between feedback messages.
     * @param canceledResult The result to return if the goal is canceled.
     */
    void configureWithFeedback(
        rclcpp_action::GoalResponse goalResponse,
        rclcpp_action::CancelResponse cancelResponse,
        std::chrono::duration<double> execTime,
        bool succeed,
        std::shared_ptr<ActionResultT> result,
        std::shared_ptr<ActionFeedbackT> feedback,
        std::chrono::duration<double> feedbackPeriod,
        std::shared_ptr<ActionResultT> canceledResult = std::make_shared<ActionResultT>())
    {
        configureNoFeedback(
            goalResponse,
            cancelResponse,
            execTime,
            succeed,
            result,
            canceledResult
        );

        this->feedbackEnabled = true;
        this->feedback = feedback;
        this->feedbackPeriod = feedbackPeriod;
    }

    /**
     * Stops execution of any goals.
     */
    void forceStop() {
        stopped = true;
    }

    /**
     * Returns whether or not the server has received a goal request since configureWithFeedback() wsa called.
     */
    bool receivedRequest() {
        return gotRequest;
    }

    /**
     * Returns whether or not the server has received a cancel request since a goal request was last received.
     */
    bool receivedCancel() {
        return gotCancel;
    }

    /**
     * @brief Returns the most recent goal request.
     * Before calling this, make sure that a goal request exists with receivedRequest()
     * 
     * @return The most recent goal request
     */
    std::shared_ptr<const ActionGoalT> getReceivedRequest() {
        return receivedGoalHandle;
    }

    protected:
    rclcpp::Node::SharedPtr node;
    typename rclcpp_action::Server<ActionT>::SharedPtr server;

    rclcpp_action::GoalResponse goalResponse;
    rclcpp_action::CancelResponse cancelResponse;
    std::chrono::duration<double> 
        execTime, 
        feedbackPeriod;
    std::shared_ptr<ActionResultT> 
        result,
        canceledResult;
    std::shared_ptr<ActionFeedbackT> feedback;
    bool 
        succeed,
        feedbackEnabled,
        gotRequest,
        gotCancel,
        stopped;

    std::shared_ptr<const ActionGoalT> receivedGoalHandle;

    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ActionGoalT> goal)
    {
        (void) uuid;
        this->gotRequest = true;
        this->gotCancel = false; //reset the cancel flag
        this->receivedGoalHandle = goal;
        return goalResponse;
    }

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandleT> goalHandle)
    {
        (void) goalHandle;
        return cancelResponse;
    }

    void handleAccepted(const std::shared_ptr<GoalHandleT> goalHandle) {
        std::thread{std::bind(&DummyActionServer::execute, this, _1), goalHandle}.detach();
    }

    //NOTE: must manually spin node in your test while this callback is happening!
    void execute(const std::shared_ptr<GoalHandleT> goalHandle) {
        rclcpp::Time startTime = node->get_clock()->now();
        while(!stopped && rclcpp::ok() && node->get_clock()->now() - startTime < execTime) {
            if(goalHandle->is_canceling() && this->cancelResponse == rclcpp_action::CancelResponse::ACCEPT) {
                goalHandle->canceled(canceledResult);
                return;
            }

            if(feedbackEnabled) {
                goalHandle->publish_feedback(feedback); //TODO: make this respect feedback period
            }
        }

        //only send results if rclcpp is ok. otherwise we will get an error that we tried to publish to a goal that doesnt exist
        if(rclcpp::ok()) {
            if(succeed) {
                goalHandle->succeed(result);
            } else {
                goalHandle->abort(result);
            }
        }
    }
};
