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


template<typename T>
class ServiceTest : public BtTest {
    typedef typename T::Request  TRequest;
    typedef typename T::Response TResponse;

    protected:
    void SetUp() override {
        BtTest::SetUp();

        serviceAllowed = true;
        requestAvailable = false;
    }

    void TearDown() override {
        BtTest::TearDown();
    }

    void configSrv(const std::string& name, std::shared_ptr<TResponse> response, std::chrono::duration<double> execTime) {
        srvThread = std::thread(
            std::bind(&ServiceTest::srvThreadFunc, this, _1, _2, _3), 
            name, 
            response, 
            execTime);
    }

    bool killSrvAndGetRequest(typename TRequest::SharedPtr request) {
        //kill the srv
        serviceAllowed = false;
        srvThread.join();

        //populate request data and return whether or not the data was available
        if(requestAvailable) {
            *request = *receivedRequest;
        }

        return requestAvailable;
    }

    private:
    void srvThreadFunc(const std::string& name, std::shared_ptr<TResponse> response, std::chrono::duration<double> execTime) {
        srvNode = std::make_shared<rclcpp::Node>("testsrv", "bt_testing");
        DummyService<T> srv(srvNode, name);
        srv.configureExecution(response, execTime);

        while(serviceAllowed) {
            rclcpp::spin_some(srvNode);
        }

        receivedRequest = srv.getReceivedRequest();
        requestAvailable = srv.requestAvailable();
    }

    rclcpp::Node::SharedPtr srvNode;
    std::thread srvThread;
    bool serviceAllowed, requestAvailable;
    typename TRequest::SharedPtr receivedRequest;
};
