#include "autonomy_test/autonomy_testing.hpp"
#include "autonomy_test/TimedPublisher.hpp"

BT::NodeStatus testGetCovariance(std::shared_ptr<BtTestTool> toolNode, const std::string& target, std::array<double, 36UL> arr, int publishIntMs, double& covariance, bool& outputSet) {
    //configure node
    BT::NodeConfiguration cfg;
    cfg.input_ports["Target"] = target;
    auto node = toolNode->createLeafNodeFromConfig("getCovariance", cfg);

    //configure publisher
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.pose.covariance = arr;
    TimedPublisher<geometry_msgs::msg::PoseWithCovarianceStamped> pub(toolNode, "mapping/" + target, msg, rclcpp::SensorDataQoS(), publishIntMs);

    //tick node until finished
    BT::NodeStatus status = toolNode->tickUntilFinished(node, 7s);

    //collect results
    outputSet = getOutputFromBlackboard<double>(toolNode, node->config().blackboard, "Covariance", covariance);
    return status;
}

TEST_F(BtTest, test_getCovariance_success_1) {
    //setup
    std::array<double, 36UL> cov = {
        1,  2,  3,  4,  5,  6,
        7,  8,  9,  10, 11, 12,
        13, 14, 15, 16, 17, 18,
        19, 20, 21, 22, 23, 24,
        25, 26, 27, 28, 29, 30,
        31, 32, 33, 34, 35, 36
    };

    //test
    double result;
    bool outputSet;
    BT::NodeStatus status = testGetCovariance(toolNode, "gman", cov, 100, result, outputSet);

    //eval
    const double expectedCov = sqrt(1 + 64 + 225 + (484 + 841 + 1296) / (2 * M_PI));
    ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(outputSet);
    ASSERT_NEAR(result, expectedCov, 0.01);
}

TEST_F(BtTest, test_getCovariance_success_2) {
    //setup
    std::array<double, 36UL> cov = {
        .1, .5, .9, .6, .8, .2,
        2,  .4, 5,  .6, .01,.4,
        1,  1,  1,  1,  1,  1,
        .5, .8, .9, .9, .1, .1,
        .3, .4, .3, .5, 2,  .1,
        .7, .8, .5, .6, .3, .9
    };

    //test
    double result;
    bool outputSet;
    BT::NodeStatus status = testGetCovariance(toolNode, "something", cov, 500, result, outputSet);

    //eval
    const double expectedCov = sqrt(.01 + .16 + 1 + (.81 + 4 + .81) / (2 * M_PI));
    ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(outputSet);
    ASSERT_NEAR(result, expectedCov, 0.01);
}

TEST_F(BtTest, test_getCovariance_fail_timed_out) {
    //setup
    std::array<double, 36UL> cov = {
        1,  2,  3,  4,  5,  6,
        7,  8,  9,  10, 11, 12,
        13, 14, 15, 16, 17, 18,
        19, 20, 21, 22, 23, 24,
        25, 26, 27, 28, 29, 30,
        31, 32, 33, 34, 35, 36
    };

    //test
    double result;
    bool outputSet;
    BT::NodeStatus status = testGetCovariance(toolNode, "gman", cov, 6000, result, outputSet);

    //eval
    ASSERT_EQ(status, BT::NodeStatus::FAILURE);
    ASSERT_FALSE(outputSet);
}
