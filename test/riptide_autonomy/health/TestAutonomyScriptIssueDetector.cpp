#include "autonomy_test/autonomy_health_util_testing.hpp"

#define SCRIPTISSUDETECTOR_FILE "scriptissuedetectortest.xml"

class AutonomyScriptIssueDetectorTest : public AutonomyHealthUtilTest
{ };


TEST_F(AutonomyScriptIssueDetectorTest, TestSimpleGoodScript)
{
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement *node = walkTree(doc, SCRIPTISSUDETECTOR_FILE,
        {
            {"BehaviorTree", 0},
            {"Sequence", 0},
            {"Script", 0}
        });
    
    ASSERT_TRUE(node);

    AutonomyScriptIssueDetector detector(
        node,
        SCRIPTISSUDETECTOR_FILE,
        _factory,
        _palette);
    
    HealthError err = detector.detect();
    printIssuesIf(detector, detector.issues().size() > 0);
    ASSERT_FALSE(err.error);
    std::vector<AutonomyIssue::Ptr> issues = detector.issues();
    ASSERT_EQ(issues.size(), 0);
    std::vector<std::string> bbDefs = detector.blackboardDefinitions();
    ASSERT_EQ(bbDefs.size(), 1);
    ASSERT_EQ(bbDefs[0], "a");
}


TEST_F(AutonomyScriptIssueDetectorTest, TestSimpleGoodScriptWithBlackboardRefs)
{
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement *node = walkTree(doc, SCRIPTISSUDETECTOR_FILE,
        {
            {"BehaviorTree", 0},
            {"Sequence", 0},
            {"Script", 1}
        });
    
    ASSERT_TRUE(node);

    std::vector<std::string> bbDefs = {
        "b",
        "c"};

    AutonomyScriptIssueDetector detector(
        node,
        SCRIPTISSUDETECTOR_FILE,
        _factory,
        _palette,
        bbDefs);
    
    HealthError err = detector.detect();
    printIssuesIf(detector, detector.issues().size() > 0);
    ASSERT_FALSE(err.error);
    std::vector<AutonomyIssue::Ptr> issues = detector.issues();
    ASSERT_EQ(issues.size(), 0);
    bbDefs = detector.blackboardDefinitions();
    ASSERT_EQ(bbDefs.size(), 3);
    ASSERT_TRUE(std::find(bbDefs.begin(), bbDefs.end(), "a") != bbDefs.end());
    ASSERT_TRUE(std::find(bbDefs.begin(), bbDefs.end(), "b") != bbDefs.end());
    ASSERT_TRUE(std::find(bbDefs.begin(), bbDefs.end(), "c") != bbDefs.end());
}


TEST_F(AutonomyScriptIssueDetectorTest, TestSimpleScriptWithBadBlackboardRefs)
{
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement *node = walkTree(doc, SCRIPTISSUDETECTOR_FILE,
        {
            {"BehaviorTree", 0},
            {"Sequence", 0},
            {"Script", 1}
        });
    
    ASSERT_TRUE(node);

    AutonomyScriptIssueDetector detector(
        node,
        SCRIPTISSUDETECTOR_FILE,
        _factory,
        _palette);
    
    HealthError err = detector.detect();
    printIssuesIf(detector, detector.issues().size() == 0);
    ASSERT_TRUE(err.error);
    std::vector<AutonomyIssue::Ptr> issues = detector.issues();
    ASSERT_EQ(issues.size(), 1);
    ASSERT_EQ(issues[0]->type(), "ScriptRuntimeError");
}


TEST_F(AutonomyScriptIssueDetectorTest, TestSimpleGoodLongScript)
{
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement *node = walkTree(doc, SCRIPTISSUDETECTOR_FILE,
        {
            {"BehaviorTree", 0},
            {"Sequence", 0},
            {"Script", 2}
        });
    
    ASSERT_TRUE(node);

    AutonomyScriptIssueDetector detector(
        node,
        SCRIPTISSUDETECTOR_FILE,
        _factory,
        _palette);
    
    HealthError err = detector.detect();
    printIssuesIf(detector, detector.issues().size() > 0);
    ASSERT_FALSE(err.error);
    std::vector<AutonomyIssue::Ptr> issues = detector.issues();
    ASSERT_EQ(issues.size(), 0);
    std::vector<std::string> bbDefs = detector.blackboardDefinitions();
    ASSERT_EQ(bbDefs.size(), 3);
    ASSERT_TRUE(std::find(bbDefs.begin(), bbDefs.end(), "haa") != bbDefs.end());
    ASSERT_TRUE(std::find(bbDefs.begin(), bbDefs.end(), "c") != bbDefs.end());
    ASSERT_TRUE(std::find(bbDefs.begin(), bbDefs.end(), "t") != bbDefs.end());
}


TEST_F(AutonomyScriptIssueDetectorTest, TestBadScript)
{
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement *node = walkTree(doc, SCRIPTISSUDETECTOR_FILE,
        {
            {"BehaviorTree", 0},
            {"Sequence", 0},
            {"Script", 3}
        });
    
    ASSERT_TRUE(node);

    AutonomyScriptIssueDetector detector(
        node,
        SCRIPTISSUDETECTOR_FILE,
        _factory,
        _palette);
    
    HealthError err = detector.detect();
    printIssuesIf(detector, detector.issues().size() == 0);
    ASSERT_TRUE(err.error);
    std::vector<AutonomyIssue::Ptr> issues = detector.issues();
    ASSERT_EQ(issues.size(), 1);
    ASSERT_EQ(issues[0]->type(), "ScriptSyntaxError");
}
