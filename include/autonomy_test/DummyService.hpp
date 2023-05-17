#pragma once
#include "autonomy_test/autonomy_testing.hpp"

using namespace std::placeholders;

template<typename T>
class DummyService {
    typedef typename T::Request TRequest;
    typedef typename T::Response TResponse;

    public:
    DummyService(rclcpp::Node::SharedPtr n, const std::string& name) {
        this->n = n;
        srv = n->create_service<T>(name, std::bind(&DummyService::srvCb, this, _1, _2));
    }


    void configureExecution(
        std::shared_ptr<TResponse> response,
        std::chrono::duration<double> execTime) 
    {
        this->response = response;
        this->execTime = execTime;
        haveRequest = false;
    }


    bool requestAvailable() {
        return haveRequest;
    }


    std::shared_ptr<TRequest> getReceivedRequest() {
        return receivedRequest;
    }


    void srvCb(const std::shared_ptr<TRequest> request, std::shared_ptr<TResponse> response) {
        haveRequest = true;
        receivedRequest = request;
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(execTime));
        *response = *this->response;
    }

    private:
    //core tools
    rclcpp::Node::SharedPtr n;
    typename rclcpp::Service<T>::SharedPtr srv;

    //execution planning
    std::shared_ptr<TResponse> response;
    std::chrono::duration<double> execTime;
    
    //status-related things
    bool haveRequest;
    std::shared_ptr<TRequest> receivedRequest;
};
