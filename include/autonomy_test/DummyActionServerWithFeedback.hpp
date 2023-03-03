#pragma once
#include "autonomy_test/autonomy_testing.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;
using Armer = riptide_msgs2::action::ArmTorpedoDropper;

/**
 * @brief This class allows for testing of action client BT nodes through very primitive execution and logging.
 * 
 * @tparam T The base action type.
 * @tparam TGoal The action goal type. should be action::Goal
 * @tparam TFeedback The feedback type. should be action::Feedback
 * @tparam TResult The action result type. should be action::Result
 */
template<typename T, typename TGoal, typename TFeedback, typename TResult>
class DummyActionServerWithFeedback {
    public:
    using TGoalHandle = rclcpp_action::ServerGoalHandle<T>;

    DummyActionServerWithFeedback(rclcpp::Node::SharedPtr rosnode, std::string serverName) {
        this->rosnode = rosnode;

        server = rclcpp_action::create_server<T>(
            rosnode,
            serverName,
            std::bind(&DummyActionServerWithFeedback::handleGoal, this, _1, _2),
            std::bind(&DummyActionServerWithFeedback::handleCancel, this, _1),
            std::bind(&DummyActionServerWithFeedback::handleAccepted, this, _1)
        );

        configureExecution(
            rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE,
            rclcpp_action::CancelResponse::ACCEPT,
            1,
            std::make_shared<TResult>(),
            std::make_shared<TFeedback>()
        );
    }

    void configureExecution(
        rclcpp_action::GoalResponse goalResponse, 
        rclcpp_action::CancelResponse cancelResponse, 
        double executionTimeSeconds,
        std::shared_ptr<TResult> result,
        std::shared_ptr<TFeedback> feedback)
         
    {
        this->goalResponse = goalResponse;
        this->cancelResponse = cancelResponse;
        this->executionTime = executionTimeSeconds;
        this->result = result;
        this->feedback = feedback;
    }

    std::shared_ptr<const TGoal> getLatestGoal() {
        return latestGoal;
    }
    const std::shared_ptr<const TFeedback> getLatestFeedback() {
        return latestFeedback;
    }

    private:
    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const TGoal> goal) {
        //get rid of unused parameter warnings
        (void)uuid;

        latestGoal = goal;
        return goalResponse;
    }

    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<TGoalHandle> goal_handle) {
        //get rid of unused parameter warnings
        (void)goal_handle;
        return cancelResponse;
    }

    void handleAccepted(const std::shared_ptr<TGoalHandle> goal_handle) {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&DummyActionServerWithFeedback::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<TGoalHandle> goalHandle) {
        if(executionTime >= 0) {
            std::chrono::duration<double> secs(executionTime);
            rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(secs));
            auto feedback = std::make_shared<Armer::Feedback>();
            feedback->is_armed = true;
            goalHandle->publish_feedback(feedback);
            
            goalHandle->succeed(result);
            return; //we done. if the code goes any further it will block until rclcpp goes bad
        }

        //okay, this part is weird. basically if the action server stays alive for too long after rclcpp goes down, a bunch of cool segfaults happen
        //in future tests. to combat this, if the action server is supposed to hang forever, it will simply sleep until rclcpp::ok() returns false
        while(rclcpp::ok()) {
            rclcpp::sleep_for(5ms);
        }
    }

    //ros node
    rclcpp::Node::SharedPtr rosnode;

    //ros action server
    std::shared_ptr<rclcpp_action::Server<T>> server;

    //execution information
    rclcpp_action::GoalResponse goalResponse;
    rclcpp_action::CancelResponse cancelResponse;
    double executionTime;
    std::shared_ptr<TResult> result;
    std::shared_ptr<TFeedback> feedback;

    //storage
    std::shared_ptr<const TGoal> latestGoal;
    const std::shared_ptr<const TFeedback> latestFeedback;
};
