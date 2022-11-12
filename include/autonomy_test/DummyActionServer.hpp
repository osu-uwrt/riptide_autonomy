#pragma once
#include "autonomy_test/autonomy_testing.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

/**
 * @brief This class allows for testing of action client BT nodes through very primitive execution and logging.
 * 
 * @tparam T The base action type.
 * @tparam TGoal The action goal type. should be action::Goal
 * @tparam TResult The action result type. should be action::Result
 */
template<typename T, typename TGoal, typename TResult>
class DummyActionServer {
    public:
    using TGoalHandle = rclcpp_action::ServerGoalHandle<T>;

    DummyActionServer(rclcpp::Node::SharedPtr rosnode, std::string serverName) {
        this->rosnode = rosnode;

        server = rclcpp_action::create_server<T>(
            rosnode,
            serverName,
            std::bind(&DummyActionServer::handleGoal, this, _1, _2),
            std::bind(&DummyActionServer::handleCancel, this, _1),
            std::bind(&DummyActionServer::handleAccepted, this, _1)
        );

        configureExecution(
            rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE,
            rclcpp_action::CancelResponse::ACCEPT,
            3s,
            std::make_shared<TResult>()
        );
    }

    void configureExecution(
        rclcpp_action::GoalResponse goalResponse, 
        rclcpp_action::CancelResponse cancelResponse, 
        std::chrono::duration<double> executionTime, 
        std::shared_ptr<TResult> result) 
    {
        this->goalResponse = goalResponse;
        this->cancelResponse = cancelResponse;
        this->executionTime = executionTime;
        this->result = result;
    }

    std::shared_ptr<const TGoal> getLatestGoal() {
        return latestGoal;
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
        std::thread{std::bind(&DummyActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<TGoalHandle> goalHandle) {
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(executionTime));
        goalHandle->succeed(result);
    }

    //ros node
    rclcpp::Node::SharedPtr rosnode;

    //ros action server
    std::shared_ptr<rclcpp_action::Server<T>> server;

    //execution information
    rclcpp_action::GoalResponse goalResponse;
    rclcpp_action::CancelResponse cancelResponse;
    std::chrono::duration<double> executionTime;
    std::shared_ptr<TResult> result;

    //storage
    std::shared_ptr<const TGoal> latestGoal;
};
