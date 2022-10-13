#include "autonomy_test/autonomy_testing.hpp"

BtTestTool::BtTestTool()
 : rclcpp::Node("BtTester") {
    factory = std::make_shared<BT::BehaviorTreeFactory>();
    registerPluginsForFactory(factory, "riptide_autonomy2");
}


BT::Blackboard::Ptr BtTestTool::runLeafNodeFromConfig(std::string name, BT::NodeConfiguration inputConfig) {
    //create a blackboard if non exists
    if(!inputConfig.blackboard) {
        inputConfig.blackboard = BT::Blackboard::create();
    }

    //configure output ports to map to blackboard entries of same name
    for(auto pair : factory->manifests().at(name).ports) {
        if(pair.second.direction() == BT::PortDirection::OUTPUT) {
            inputConfig.output_ports[pair.first] = pair.first; //maps a blackboard entry to output port. both have same name.
        }
    }

    auto node = factory->instantiateTreeNode("Test", name, inputConfig);
    node->executeTick();
    
    return node->config().blackboard;
}
