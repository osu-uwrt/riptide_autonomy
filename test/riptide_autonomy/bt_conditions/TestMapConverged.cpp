#include "autonomy_test/autonomy_testing.hpp"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

using namespace std::chrono_literals;

namespace {
geometry_msgs::msg::PoseWithCovarianceStamped estimate(double variance) {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    for(int i = 0; i < 3; i++) {
        msg.pose.covariance[7 * i] = variance;
    }
    return msg;
}

std::shared_ptr<BT::TreeNode> mapConverged(std::shared_ptr<BtTestTool> tool, const std::string& object, double target) {
    BT::NodeConfiguration cfg;
    cfg.input_ports["object"] = object;
    cfg.input_ports["target_cov"] = std::to_string(target);
    return tool->createLeafNodeFromConfig("MapConverged", cfg);
}
}

TEST_F(BtTest, test_MapConverged_fails_before_any_estimate) {
    auto node = mapConverged(toolNode, "torpedo", 0.01);
    ASSERT_EQ(node->executeTick(), BT::NodeStatus::FAILURE);
}

TEST_F(BtTest, test_MapConverged_follows_the_estimate) {
    auto node = mapConverged(toolNode, "torpedo", 0.01);
    auto pub = toolNode->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("mapping/torpedo", 10);
    ASSERT_EQ(node->executeTick(), BT::NodeStatus::FAILURE); // subscribes
    toolNode->spinForTime(0.3s); // discovery
    node->executeTick();
    pub->publish(estimate(0.5)); // error sqrt(3) * 0.5
    toolNode->spinForTime(0.2s);
    ASSERT_EQ(node->executeTick(), BT::NodeStatus::FAILURE);
    pub->publish(estimate(0.001)); // error 0.0017
    toolNode->spinForTime(0.2s);
    ASSERT_EQ(node->executeTick(), BT::NodeStatus::SUCCESS);
}

TEST_F(BtTest, test_MapConverged_ignores_estimates_from_before_the_check) {
    auto node = mapConverged(toolNode, "bin", 0.01);
    auto pub = toolNode->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("mapping/bin", 10);
    node->executeTick();
    toolNode->spinForTime(0.3s);
    node->executeTick();
    pub->publish(estimate(0.001));
    toolNode->spinForTime(0.2s);
    ASSERT_EQ(node->executeTick(), BT::NodeStatus::SUCCESS);
    // Not ticked for over a second (the tree moved on, maybe reset the target): a new check needs a new estimate
    toolNode->spinForTime(1.2s);
    ASSERT_EQ(node->executeTick(), BT::NodeStatus::FAILURE);
    pub->publish(estimate(0.001));
    toolNode->spinForTime(0.2s);
    ASSERT_EQ(node->executeTick(), BT::NodeStatus::SUCCESS);
}
