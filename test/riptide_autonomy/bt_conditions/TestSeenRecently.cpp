#include "autonomy_test/autonomy_testing.hpp"

#include <vision_msgs/msg/detection3_d_array.hpp>

using namespace std::chrono_literals;

namespace {
vision_msgs::msg::Detection3DArray detection(const std::string& classId, double score) {
    vision_msgs::msg::ObjectHypothesisWithPose result;
    result.hypothesis.class_id = classId;
    result.hypothesis.score = score;
    vision_msgs::msg::Detection3D det;
    det.results.push_back(result);
    vision_msgs::msg::Detection3DArray msg;
    msg.detections.push_back(det);
    return msg;
}

std::shared_ptr<BT::TreeNode> seenRecently(std::shared_ptr<BtTestTool> tool, const std::string& object, double maxAge) {
    BT::NodeConfiguration cfg;
    cfg.input_ports["object_name"] = object;
    cfg.input_ports["max_age_secs"] = std::to_string(maxAge);
    return tool->createLeafNodeFromConfig("SeenRecently", cfg);
}
}

TEST_F(BtTest, test_SeenRecently_fails_before_any_detection) {
    auto node = seenRecently(toolNode, "bin", 0.5);
    ASSERT_EQ(node->executeTick(), BT::NodeStatus::FAILURE);
}

TEST_F(BtTest, test_SeenRecently_succeeds_while_fresh_then_expires) {
    auto node = seenRecently(toolNode, "bin", 0.5);
    auto pub = toolNode->create_publisher<vision_msgs::msg::Detection3DArray>(DETECTIONS_TOPIC, 10);
    toolNode->spinForTime(0.3s); // discovery
    pub->publish(detection("bin", 0.9));
    toolNode->spinForTime(0.2s);
    ASSERT_EQ(node->executeTick(), BT::NodeStatus::SUCCESS);
    toolNode->spinForTime(0.8s); // older than max_age_secs
    ASSERT_EQ(node->executeTick(), BT::NodeStatus::FAILURE);
}

TEST_F(BtTest, test_SeenRecently_ignores_other_objects_and_low_confidence) {
    auto node = seenRecently(toolNode, "bin", 1.0);
    auto pub = toolNode->create_publisher<vision_msgs::msg::Detection3DArray>(DETECTIONS_TOPIC, 10);
    toolNode->spinForTime(0.3s);
    pub->publish(detection("table", 0.9));
    pub->publish(detection("bin", 0.1)); // below mapping's 0.2 confidence cutoff
    toolNode->spinForTime(0.2s);
    ASSERT_EQ(node->executeTick(), BT::NodeStatus::FAILURE);
}
