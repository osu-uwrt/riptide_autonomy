#include "autonomy.h"

using namespace std::chrono_literals;

BT::NodeStatus RetryUntilSuccessfulOrTimeout::tick() {
    auto startTime = std::chrono::system_clock::now();
    double timeoutDuration = getInput<double>("num_seconds").value();
    
    auto chronoDuration = std::chrono::duration<double>(timeoutDuration);
    while((std::chrono::system_clock::now() - startTime) < chronoDuration) {
        NodeStatus result = child()->executeTick();

        if(result == NodeStatus::SUCCESS) {
            return NodeStatus::SUCCESS;
        }
    }

    return NodeStatus::FAILURE;
}
