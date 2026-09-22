#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    const char *domain = std::getenv("ROS_DOMAIN_ID");
    if (!domain || std::string(domain) == "0") {
        std::cerr << "Run this regression in an isolated ROS_DOMAIN_ID\n";
        return 1;
    }
    rclcpp::init(argc, argv);
    using Trigger = std_srvs::srv::Trigger;
    using Odometry = nav_msgs::msg::Odometry;
    auto node = std::make_shared<rclcpp::Node>("client", "/service_executor_test");
    auto server = std::make_shared<rclcpp::Node>("server", "/service_executor_test");
    std::atomic<int> handled{0}, callbacks{0};
    std::array<rclcpp::Service<Trigger>::SharedPtr, 3> services;
    std::array<rclcpp::Client<Trigger>::SharedPtr, 3> clients;
    for (size_t i = 0; i < clients.size(); ++i) {
        const std::string name = "probe_" + std::to_string(i);
        services[i] = server->create_service<Trigger>(name,
            [&handled, name](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
                ++handled;
                response->success = true;
                response->message = name;
            });
        clients[i] = node->create_client<Trigger>(name);
    }
    auto publisher = server->create_publisher<Odometry>("odometry", 10);
    auto timer = server->create_wall_timer(10ms, [&] { publisher->publish(Odometry{}); });
    auto heartbeat = node->create_wall_timer(1s, [] {});
    std::vector<rclcpp::Subscription<Odometry>::SharedPtr> subscriptions;
    // Reproduce a busy default callback group without expensive sensor data.
    // The bounded work exceeds the incoming subscription period in aggregate,
    // but a ready service response must still get an executor turn. Mission
    // subtrees can instantiate hundreds of idle GetOdometry subscriptions.
    for (int i = 0; i < 31; ++i) {
        subscriptions.push_back(node->create_subscription<Odometry>("odometry", 10,
            [&](Odometry::SharedPtr) {
                ++callbacks;
                std::this_thread::sleep_for(500us);
            }));
    }
    rclcpp::executors::SingleThreadedExecutor executor;
    rclcpp::executors::SingleThreadedExecutor serverExecutor;
    executor.add_node(node);
    serverExecutor.add_node(server);
    std::thread clientThread([&] { executor.spin(); });
    std::thread serverThread([&] { serverExecutor.spin(); });

    bool passed = true;
    for (const auto &client : clients)
        passed &= client->wait_for_service(5s);
    const auto discoveryDeadline = std::chrono::steady_clock::now() + 5s;
    while (publisher->get_subscription_count() < subscriptions.size() &&
           std::chrono::steady_clock::now() < discoveryDeadline)
        std::this_thread::sleep_for(10ms);
    passed &= publisher->get_subscription_count() == subscriptions.size();
    std::this_thread::sleep_for(500ms);

    int completed = 0;
    double worstMs = 0;
    constexpr int requests = 36;
    if (passed) {
        for (int i = 0; i < requests; ++i) {
            const auto &client = clients[i % clients.size()];
            const auto start = std::chrono::steady_clock::now();
            auto future = client->async_send_request(std::make_shared<Trigger::Request>());
            if (future.wait_for(1s) == std::future_status::ready) {
                const auto response = future.get();
                passed &= response->success &&
                    response->message == "probe_" + std::to_string(i % clients.size());
                ++completed;
            } else {
                client->remove_pending_request(future);
                passed = false;
            }
            worstMs = std::max(worstMs, std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count());
            std::this_thread::sleep_for(30ms);
        }
    }
    executor.cancel();
    serverExecutor.cancel();
    clientThread.join();
    serverThread.join();
    rclcpp::shutdown();
    std::cout << "Handled " << handled << ", completed " << completed << "/" << requests
              << "; worst response " << worstMs << " ms; subscription callbacks "
              << callbacks << '\n';
    return passed && handled == requests && completed == requests && callbacks > 100 ? 0 : 1;
}
