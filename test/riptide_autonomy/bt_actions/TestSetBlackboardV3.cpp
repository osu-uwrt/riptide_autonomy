#include "autonomy_test/autonomy_testing.hpp"

TEST_F(BtTest, test_SetBlackboardV3_stores_text) {
    BT::NodeConfiguration in;
    in.blackboard = BT::Blackboard::create();
    in.input_ports["output_key"] = "depth";
    in.input_ports["value"] = "-1.5";
    auto node = toolNode->createLeafNodeFromConfig("SetBlackboardV3", in);

    ASSERT_EQ(toolNode->tickUntilFinished(node, 5s), BT::NodeStatus::SUCCESS);
    EXPECT_EQ(in.blackboard->get<std::string>("depth"), "-1.5");
}

TEST_F(BtTest, test_SetBlackboardV3_missing_value_names_the_port) {
    BT::NodeConfiguration in;
    in.blackboard = BT::Blackboard::create();
    in.input_ports["output_key"] = "depth";
    auto node = toolNode->createLeafNodeFromConfig("SetBlackboardV3", in);

    try {
        node->executeTick();
        FAIL() << "expected a RuntimeError";
    } catch(const BT::RuntimeError& e) {
        EXPECT_NE(std::string(e.what()).find("missing port [value]"), std::string::npos) << e.what();
    }
}
