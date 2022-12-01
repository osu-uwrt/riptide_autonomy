#pragma once

#include "autonomy_lib.hpp"

class SimpleActions {
    public:
    static void bulkRegister(BT::BehaviorTreeFactory &factory);
};

class SimpleConditions {
    public:
    static void bulkRegister(BT::BehaviorTreeFactory &factory);
};

class ActuatorConditions {
    public:
    static void bulkRegister(BT::BehaviorTreeFactory &factory);
};
