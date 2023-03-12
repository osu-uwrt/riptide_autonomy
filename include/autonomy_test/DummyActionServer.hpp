#pragma once
#include "autonomy_test/autonomy_testing.hpp"

using namespace std::placeholders;

template<typename ActionT>
class DummyActionServer {
    public:
    typedef rclcpp_action::ServerGoalHandle<ActionT> GoalHandleT;
    typedef typename ActionT::Goal ActionGoalT;
    typedef typename ActionT::Feedback ActionFeedbackT;
    typedef typename ActionT::Result ActionResultT;

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
    }

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
    }

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

    bool receivedRequest() {
        return gotRequest;
    }

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
        gotRequest;

    std::shared_ptr<const ActionGoalT> receivedGoalHandle;

    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ActionGoalT> goal)
    {
        (void) uuid;
        this->gotRequest = true;
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
        while(rclcpp::ok() && node->get_clock()->now() - startTime < execTime) {
            if(goalHandle->is_canceling() && this->cancelResponse == rclcpp_action::CancelResponse::ACCEPT) {
                goalHandle->canceled(canceledResult);
                return;
            }

            if(feedbackEnabled) {
                goalHandle->publish_feedback(feedback);
            }
        }

        if(rclcpp::ok()) {
            if(succeed) {
                goalHandle->succeed(result);
            } else {
                goalHandle->abort(result);
            }
        }
    }
};
