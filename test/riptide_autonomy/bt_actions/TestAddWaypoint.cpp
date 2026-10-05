#include "autonomy_test/autonomy_testing.hpp"

std::string addWaypointTest(std::shared_ptr<BtTestTool> toolNode, const std::string& path, const std::string& frame) {
    BT::NodeConfiguration cfg;
    if(!path.empty()) {
        cfg.input_ports["path"] = path;
    }
    if(!frame.empty()) {
        cfg.input_ports["frame"] = frame;
    }
    cfg.input_ports["x"] = "1";
    cfg.input_ports["y"] = "-2.5";
    cfg.input_ports["z"] = "0";
    cfg.input_ports["yaw"] = "3.1415";

    auto node = toolNode->createLeafNodeFromConfig("AddWaypoint", cfg);
    EXPECT_EQ(toolNode->tickUntilFinished(node), BT::NodeStatus::SUCCESS);

    std::string out;
    EXPECT_TRUE(getOutputFromBlackboard<std::string>(toolNode, node->config().blackboard, "out", out));
    return out;
}

TEST_F(BtTest, test_AddWaypoint_new_path) {
    ASSERT_EQ(addWaypointTest(toolNode, "", ""), "1,-2.5,0,3.1415;");
}

TEST_F(BtTest, test_AddWaypoint_new_path_with_frame) {
    ASSERT_EQ(addWaypointTest(toolNode, "", "gate_frame"), "gate_frame: 1,-2.5,0,3.1415;");
}

TEST_F(BtTest, test_AddWaypoint_append) {
    ASSERT_EQ(addWaypointTest(toolNode, "a: 0,0,0,0;", "b"), "a: 0,0,0,0; b: 1,-2.5,0,3.1415;");
}

TEST_F(BtTest, test_AddWaypoint_append_without_semicolon) {
    ASSERT_EQ(addWaypointTest(toolNode, "0,0,0,0", ""), "0,0,0,0; 1,-2.5,0,3.1415;");
}

TEST_F(BtTest, test_AddWaypoint_heading_options) {
    BT::NodeConfiguration cfg;
    cfg.input_ports["frame"] = "g";
    cfg.input_ports["x"] = "1";
    cfg.input_ports["y"] = "2";
    cfg.input_ports["z"] = "-1";
    cfg.input_ports["heading"] = "look_at";
    cfg.input_ports["look_x"] = "4";
    cfg.input_ports["spin"] = "6.283";
    auto node = toolNode->createLeafNodeFromConfig("AddWaypoint", cfg);
    ASSERT_EQ(toolNode->tickUntilFinished(node), BT::NodeStatus::SUCCESS);
    std::string out;
    ASSERT_TRUE(getOutputFromBlackboard<std::string>(toolNode, node->config().blackboard, "out", out));
    ASSERT_EQ(out, "g: 1,2,-1,0 | heading=look_at | look_at=4,0,0 | spin=6.283;");
}

TEST_F(BtTest, test_AddWaypoint_bad_heading) {
    BT::NodeConfiguration cfg;
    cfg.input_ports["x"] = "1";
    cfg.input_ports["y"] = "2";
    cfg.input_ports["z"] = "-1";
    cfg.input_ports["heading"] = "sideways";
    auto node = toolNode->createLeafNodeFromConfig("AddWaypoint", cfg);
    ASSERT_EQ(toolNode->tickUntilFinished(node), BT::NodeStatus::FAILURE);
}

TEST_F(BtTest, test_AddArc_approach_then_arc_looking_at_centre) {
    BT::NodeConfiguration cfg;
    cfg.input_ports["path"] = "a: 0,0,0,0;";
    cfg.input_ports["frame"] = "g";
    cfg.input_ports["cx"] = "-10";
    cfg.input_ports["cy"] = "0";
    cfg.input_ports["radius"] = "3";
    cfg.input_ports["start_angle"] = "0";
    cfg.input_ports["sweep"] = "1.5707963267948966";
    cfg.input_ports["z"] = "-1.5";
    cfg.input_ports["heading"] = "look_at";
    auto node = toolNode->createLeafNodeFromConfig("AddArc", cfg);
    ASSERT_EQ(toolNode->tickUntilFinished(node), BT::NodeStatus::SUCCESS);
    std::string out;
    ASSERT_TRUE(getOutputFromBlackboard<std::string>(toolNode, node->config().blackboard, "out", out));
    // A line to (-7, 0), then a quarter turn round (-10, 0) to (-10, 3), facing the centre throughout.
    const std::string start = "a: 0,0,0,0; g: -7,0,-1.5,0 | heading=look_at | look_at=-10,0,-1.5; g: -10,";
    ASSERT_EQ(out.substr(0, start.size()), start);
    ASSERT_NE(out.find(",3,-1.5,0 | arc=-10,0,1.570796327 | heading=look_at | look_at=-10,0,-1.5;"), std::string::npos) << out;
}

// Groot saves unset ports as "": they must mean the defaults.
TEST_F(BtTest, test_AddWaypoint_empty_ports_are_defaults) {
    BT::NodeConfiguration cfg;
    for (const char *port : {"path", "frame", "yaw", "heading", "look_x", "look_y", "look_z", "yaw_offset", "spin", "spin_rate"})
        cfg.input_ports[port] = "";
    cfg.input_ports["x"] = "1";
    cfg.input_ports["y"] = "2";
    cfg.input_ports["z"] = "-1";
    auto node = toolNode->createLeafNodeFromConfig("AddWaypoint", cfg);
    ASSERT_EQ(toolNode->tickUntilFinished(node), BT::NodeStatus::SUCCESS);
    std::string out;
    ASSERT_TRUE(getOutputFromBlackboard<std::string>(toolNode, node->config().blackboard, "out", out));
    ASSERT_EQ(out, "1,2,-1,0;");
}

// look_frame: faces that frame's origin (followed live by the controller), heading look_at implied.
TEST_F(BtTest, test_AddArc_look_frame_defaults_to_its_origin) {
    BT::NodeConfiguration cfg;
    cfg.input_ports["frame"] = "g";
    cfg.input_ports["cx"] = "-10";
    cfg.input_ports["cy"] = "0";
    cfg.input_ports["radius"] = "3";
    cfg.input_ports["start_angle"] = "0";
    cfg.input_ports["sweep"] = "1.5707963267948966";
    cfg.input_ports["z"] = "-1.5";
    cfg.input_ports["look_frame"] = "pole_frame";
    auto node = toolNode->createLeafNodeFromConfig("AddArc", cfg);
    ASSERT_EQ(toolNode->tickUntilFinished(node), BT::NodeStatus::SUCCESS);
    std::string out;
    ASSERT_TRUE(getOutputFromBlackboard<std::string>(toolNode, node->config().blackboard, "out", out));
    const std::string look = " | heading=look_at | look_at=0,0,0 | look_frame=pole_frame;";
    ASSERT_EQ(out.find("g: -7,0,-1.5,0" + look), 0u) << out;
    ASSERT_NE(out.find("| arc=-10,0,1.570796327" + look), std::string::npos) << out;
}

TEST_F(BtTest, test_AddWaypoint_look_frame_needs_look_at) {
    BT::NodeConfiguration cfg;
    cfg.input_ports["x"] = "1";
    cfg.input_ports["y"] = "2";
    cfg.input_ports["z"] = "-1";
    cfg.input_ports["heading"] = "path";
    cfg.input_ports["look_frame"] = "pole_frame";
    auto node = toolNode->createLeafNodeFromConfig("AddWaypoint", cfg);
    ASSERT_EQ(toolNode->tickUntilFinished(node), BT::NodeStatus::FAILURE);
}
