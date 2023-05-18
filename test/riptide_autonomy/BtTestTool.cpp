#include "autonomy_test/autonomy_testing.hpp"

using namespace std::chrono_literals;

BtTestTool::BtTestTool()
 : rclcpp::Node("BtTester", "bt_testing") {
    factory = std::make_shared<BT::BehaviorTreeFactory>();
    registerPluginsForFactory(factory, "riptide_autonomy2");
}


std::shared_ptr<BT::TreeNode> BtTestTool::createLeafNodeFromConfig(std::string name, BT::NodeConfiguration inputConfig) {
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

    //initialize node with ROS context if appropriate (yes, single equal sign, not double)
    if(auto btNode = dynamic_cast<UwrtBtNode *>(node.get())) {
        btNode->init(this->shared_from_this());
    }

    return node;
}


std::shared_ptr<BT::TreeNode> BtTestTool::createDecoratorNodeFromConfig(std::string name, BT::NodeConfiguration inputConfig, BT::TreeNode::Ptr child) {
    auto node = createLeafNodeFromConfig(name, inputConfig);

    if(auto decorator = dynamic_cast<BT::DecoratorNode *>(node.get())) {
        decorator->setChild(child.get());
        return node;
    }

    //its not a decorator!
    std::string msg = "Node with name " + name + " is not a decorator.";
    RCLCPP_ERROR(log, msg.c_str());
    throw std::runtime_error(msg);
}


std::shared_ptr<DummyActionNode> BtTestTool::createDummyActionNode() {
    auto ptr = std::make_shared<DummyActionNode>("Dummy", BT::NodeConfiguration());
    ptr->init(this->shared_from_this());
    return ptr;
}


BT::NodeStatus BtTestTool::tickUntilFinished(std::shared_ptr<BT::TreeNode> node) {
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    while(status != BT::NodeStatus::SUCCESS && status != BT::NodeStatus::FAILURE) {
        rclcpp::spin_some(shared_from_this()); //spin should come before executing tick for things like action servers that need to go online
        status = node->executeTick();
    }

    return status;
}

void BtTestTool::spinForTime(std::chrono::duration<double> time) {
    auto startTime = this->get_clock()->now();
    while(rclcpp::ok() && this->get_clock()->now() - startTime < time) {
        rclcpp::spin_some(shared_from_this());
    }
}
