#include "riptide_autonomy/autonomy_health.hpp"

//
// use this file to define the node execution characteristics for BT control nodes
//

const NodeExecutionOrder SEQUENTIAL_NODE_EXECUTION_ORDER = [] (size_t n)
{
    std::vector<int> s;
    for(size_t i = 0; i < n; i++)
    {
        s.push_back(i);
    }

    return s;
};

const NodeExecutionDescription SEQUENTIAL_LINKED_NODE_EXECUTION_DESCRIPTION = {
    NodeExecutionOrderWithBlackboard(SEQUENTIAL_NODE_EXECUTION_ORDER, BLACKBOARD_LINKED)
};

const NodeExecutionDescription SEQUENTIAL_UNLINKED_NODE_EXECUTION_DESCRIPTION = {
    NodeExecutionOrderWithBlackboard(SEQUENTIAL_NODE_EXECUTION_ORDER, BLACKBOARD_UNLINKED)
};

const NodeExecutionDescription CONDITION_DRIVEN_NODE_EXECUTION_DESCRIPTION = {
    NodeExecutionOrderWithBlackboard([](size_t n) { return std::vector<int>{ 1 }; }, BLACKBOARD_LINKED),
    NodeExecutionOrderWithBlackboard([](size_t n) { return std::vector<int>{ 2, 3 }; }, BLACKBOARD_UNLINKED),
};


const std::map<std::string, NodeExecutionDescription> NODE_EXECUTION_DESCRIPTIONS()
{
    return {
        {
            "AsyncFallback",
            SEQUENTIAL_LINKED_NODE_EXECUTION_DESCRIPTION
        },
        {
            "AsyncSequence",
            SEQUENTIAL_LINKED_NODE_EXECUTION_DESCRIPTION
        },
        {
            "Fallback",
            SEQUENTIAL_LINKED_NODE_EXECUTION_DESCRIPTION
        },
        {
            "IfThenElse",
            CONDITION_DRIVEN_NODE_EXECUTION_DESCRIPTION
        },
        {
            "Parallel",
            SEQUENTIAL_UNLINKED_NODE_EXECUTION_DESCRIPTION
        },
        {
            "ParallelAll",
            SEQUENTIAL_UNLINKED_NODE_EXECUTION_DESCRIPTION
        },
        {
            "ReactiveFallback",
            SEQUENTIAL_LINKED_NODE_EXECUTION_DESCRIPTION
        },
        {
            "ReactiveSequence",
            SEQUENTIAL_LINKED_NODE_EXECUTION_DESCRIPTION
        },
        {
            "SequenceWithMemory",
            SEQUENTIAL_LINKED_NODE_EXECUTION_DESCRIPTION
        },
        {
            "Switch2",
            SEQUENTIAL_UNLINKED_NODE_EXECUTION_DESCRIPTION
        },
        {
            "Switch3",
            SEQUENTIAL_UNLINKED_NODE_EXECUTION_DESCRIPTION
        },
        {
            "Switch4",
            SEQUENTIAL_UNLINKED_NODE_EXECUTION_DESCRIPTION
        },
        {
            "Switch5",
            SEQUENTIAL_UNLINKED_NODE_EXECUTION_DESCRIPTION
        },
        {
            "Switch6",
            SEQUENTIAL_UNLINKED_NODE_EXECUTION_DESCRIPTION
        },
        {
            "WhileDoElse",
            CONDITION_DRIVEN_NODE_EXECUTION_DESCRIPTION
        }
    };
}
