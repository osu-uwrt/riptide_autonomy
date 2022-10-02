#include "bt_decorators/RetryUntilSuccessfulOrTimeout.h"

using namespace std::chrono_literals;


BT::PortsList RetryUntilSuccessfulOrTimeout::providedPorts() {
    return {
        BT::InputPort<double>("num_seconds")
    };
}


BT::NodeStatus RetryUntilSuccessfulOrTimeout::tick() {
    auto startTime = std::chrono::system_clock::now();
    double timeoutDuration = getInput<double>("num_seconds").value();
    
    auto chronoDuration = std::chrono::duration<double>(timeoutDuration);
    while((std::chrono::system_clock::now() - startTime) < chronoDuration) {
        BT::NodeStatus result = child()->executeTick();

        if(result == BT::NodeStatus::SUCCESS) {
            return BT::NodeStatus::SUCCESS;
        }
    }

    return BT::NodeStatus::FAILURE;
}
