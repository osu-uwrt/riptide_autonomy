#pragma once

#include "riptide_autonomy/autonomy_health.hpp"
#include <gtest/gtest.h>


class AutonomyHealthUtilTest : public ::testing::Test
{
    protected:
    void SetUp() override;
    void TearDown() override;
    tinyxml2::XMLElement *walkTree(tinyxml2::XMLDocument& doc, const std::string& file, const std::vector<std::pair<std::string, int>>& path);
    void printIssuesIf(const AutonomyIssueDetector& detector, bool condition);

    std::shared_ptr<BT::BehaviorTreeFactory> _factory;
    NodeManifests _palette;
};
