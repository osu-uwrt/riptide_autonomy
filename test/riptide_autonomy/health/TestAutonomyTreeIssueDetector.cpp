#include "autonomy_test/autonomy_health_util_testing.hpp"

#define TREEISSUEDETECTOR_FILE "treeissuedetectortest.xml"
#define BADINCLUDE_FILE "treeissuedetector_bad_include.xml"
#define EMPTYTREE_FILE "treeissuedetector_empty_tree.xml"
#define BADNODECASES_FILE "treeissuedetector_bad_node_cases.xml"

class AutonomyTreeIssueDetectorTest : public AutonomyHealthUtilTest
{ };


TEST_F(AutonomyTreeIssueDetectorTest, TestSimpleGoodTree)
{
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement *node = walkTree(doc, TREEISSUEDETECTOR_FILE, 
        {
            {"BehaviorTree", 0}
        });
    
    ASSERT_TRUE(node);

    AutonomyTreeIssueDetector treeIssueDetector(TREEISSUEDETECTOR_FILE, "~", node, _factory, _palette);
    HealthError err = treeIssueDetector.detect();
    printIssuesIf(treeIssueDetector, treeIssueDetector.issues().size() > 0);
    ASSERT_FALSE(err.error);
    std::vector<AutonomyIssue::Ptr> issues = treeIssueDetector.issues();
    ASSERT_EQ(issues.size(), 0);
}


TEST_F(AutonomyTreeIssueDetectorTest, TestComplexGoodTree)
{
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement *node = walkTree(doc, TREEISSUEDETECTOR_FILE, 
        {
            {"BehaviorTree", 2}
        });
    
    ASSERT_TRUE(node);

    AutonomyTreeIssueDetector treeIssueDetector(TREEISSUEDETECTOR_FILE, "~", node, _factory, _palette);
    HealthError err = treeIssueDetector.detect();
    printIssuesIf(treeIssueDetector, treeIssueDetector.issues().size() > 0);
    ASSERT_FALSE(err.error);
    std::vector<AutonomyIssue::Ptr> issues = treeIssueDetector.issues();
    ASSERT_EQ(issues.size(), 0);
}


TEST_F(AutonomyTreeIssueDetectorTest, TestRecursiveNodeIssueDetection)
{
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement *node = walkTree(doc, TREEISSUEDETECTOR_FILE, 
        {
            {"BehaviorTree", 1}
        });
    
    ASSERT_TRUE(node);

    _palette.insert({"Info", BT::TreeNodeManifest()});

    AutonomyTreeIssueDetector treeIssueDetector(TREEISSUEDETECTOR_FILE, "~", node, _factory, _palette);
    HealthError err = treeIssueDetector.detect();
    printIssuesIf(treeIssueDetector, treeIssueDetector.issues().size() != 1);
    ASSERT_FALSE(err.error);
    std::vector<AutonomyIssue::Ptr> issues = treeIssueDetector.issues();
    ASSERT_EQ(issues.size(), 1);
    ASSERT_EQ(issues[0]->type(), "RequiredPortError");
}


TEST_F(AutonomyTreeIssueDetectorTest, TestBadInclude)
{
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement *node = walkTree(doc, BADINCLUDE_FILE, { });
    
    ASSERT_TRUE(node);

    AutonomyTreeIssueDetector treeIssueDetector(BADINCLUDE_FILE, "~", node, _factory, _palette);
    HealthError err = treeIssueDetector.detect();
    printIssuesIf(treeIssueDetector, treeIssueDetector.issues().size() != 1);
    ASSERT_FALSE(err.error);
    std::vector<AutonomyIssue::Ptr> issues = treeIssueDetector.issues();
    ASSERT_EQ(issues.size(), 1);
    ASSERT_EQ(issues[0]->type(), "UnspecifiedIncludeError");
}


TEST_F(AutonomyTreeIssueDetectorTest, TestEmptyTree)
{
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement *node = walkTree(doc, EMPTYTREE_FILE, { });
    
    ASSERT_TRUE(node);

    AutonomyTreeIssueDetector treeIssueDetector(EMPTYTREE_FILE, "~", node, _factory, _palette);
    HealthError err = treeIssueDetector.detect();
    printIssuesIf(treeIssueDetector, treeIssueDetector.issues().size() != 1);
    ASSERT_FALSE(err.error);
    std::vector<AutonomyIssue::Ptr> issues = treeIssueDetector.issues();
    ASSERT_EQ(issues.size(), 1);
    ASSERT_EQ(issues[0]->type(), "EmptyTree");
}


TEST_F(AutonomyTreeIssueDetectorTest, TestBadNodeTypes)
{
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement *node = walkTree(doc, BADNODECASES_FILE, { });
    
    ASSERT_TRUE(node);

    AutonomyTreeIssueDetector treeIssueDetector(BADNODECASES_FILE, "~", node, _factory, _palette);
    HealthError err = treeIssueDetector.detect();
    printIssuesIf(treeIssueDetector, treeIssueDetector.issues().size() != 3);
    ASSERT_FALSE(err.error);
    std::vector<AutonomyIssue::Ptr> issues = treeIssueDetector.issues();
    ASSERT_EQ(issues.size(), 3);
    ASSERT_TRUE(issueVectorContains(issues, "BTControlError"));
    ASSERT_TRUE(issueVectorContains(issues, "BTDecoratorError"));
    ASSERT_TRUE(issueVectorContains(issues, "BTLeafError"));
}
