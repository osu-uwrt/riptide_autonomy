#include "autonomy_test/autonomy_testing.hpp"
//will also #include whatever node is being tested, which will #include autonomy_lib.

// #include "riptide_autonomy/autonomy_lib.hpp"

TEST(BtTest, dummy_test) {
    EXPECT_EQ(2 * 2,  4);
}

TEST(BtTest, node_execution_test) {
    BT::NodeConfiguration in;
    in.input_ports["a"] = "2";
    in.input_ports["b"] = "2";
    in.input_ports["operator"] = "+";

    BT::Blackboard::Ptr out = BtTestEnvironment::getBtTestTool()->runLeafNodeFromConfig("Math", in);
    
    int defaultVal = 0;
    ASSERT_EQ(getFromBlackboardWithDefault<int>(out, "out", defaultVal), 4);
}
