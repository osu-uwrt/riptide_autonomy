#include "autonomy_test/autonomy_health_util_testing.hpp"

#define NODEISSUDETECTOR_FILE "nodeissuedetectortest.xml"
#define BADMANIFEST_FILE "nodeissuedetectortest_bad_manifest.xml"

class AutonomyNodeIssueDetectorTest : public AutonomyHealthUtilTest
{ };


TEST_F(AutonomyNodeIssueDetectorTest, TestGoodBuiltinNodeWithoutPorts)
{
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement *node = walkTree(doc, NODEISSUDETECTOR_FILE,
        {
            {"BehaviorTree", 2},
            {"Sequence", 0},
            {"Fallback", 0}
        });
    
    ASSERT_TRUE(node);

    //add our nodes to the model
    _palette.insert({ "CallSetBoolService", BT::TreeNodeManifest() });
    _palette.insert({ "CallTriggerService", BT::TreeNodeManifest() });
    _palette.insert({ "CompareNums", BT::TreeNodeManifest() });
    _palette.insert({ "Info", BT::TreeNodeManifest() });

    AutonomyNodeIssueDetector nodeIssueDetector(node, NODEISSUDETECTOR_FILE, _factory, _palette);
    HealthError err = nodeIssueDetector.detect();
    printIssuesIf(nodeIssueDetector, nodeIssueDetector.issues().size() > 0);
    ASSERT_FALSE(err.error);
    std::vector<AutonomyIssue::Ptr> issues = nodeIssueDetector.issues();
    ASSERT_EQ(issues.size(), 0);
}

TEST_F(AutonomyNodeIssueDetectorTest, TestGoodBuiltinNodeWithPorts)
{
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement *node = walkTree(doc, NODEISSUDETECTOR_FILE,
        {
            {"BehaviorTree", 2},
            {"Sequence", 0},
            {"Script", 0}
        });
    
    ASSERT_TRUE(node);

    //add our nodes to the model
    _palette.insert({ "CallSetBoolService", BT::TreeNodeManifest() });
    _palette.insert({ "CallTriggerService", BT::TreeNodeManifest() });
    _palette.insert({ "CompareNums", BT::TreeNodeManifest() });
    _palette.insert({ "Info", BT::TreeNodeManifest() });

    AutonomyNodeIssueDetector nodeIssueDetector(node, NODEISSUDETECTOR_FILE, _factory, _palette);
    HealthError err = nodeIssueDetector.detect();
    printIssuesIf(nodeIssueDetector, nodeIssueDetector.issues().size() != 0);
    ASSERT_FALSE(err.error);
    std::vector<AutonomyIssue::Ptr> issues = nodeIssueDetector.issues();
    ASSERT_EQ(issues.size(), 0);
    
    std::vector<std::string> bbDefs = nodeIssueDetector.blackboardDefinitions();
    ASSERT_EQ(bbDefs.size(), 1);
    ASSERT_EQ(bbDefs[0], "var");
}


TEST_F(AutonomyNodeIssueDetectorTest, TestSubtreeWithoutPorts)
{
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement *node = walkTree(doc, NODEISSUDETECTOR_FILE,
        {
            {"BehaviorTree", 2},
            {"Sequence", 0},
            {"SubTree", 0}
        });
    
    ASSERT_TRUE(node);

    //add our nodes to the model
    _palette.insert({ "CallSetBoolService", BT::TreeNodeManifest() });
    _palette.insert({ "CallTriggerService", BT::TreeNodeManifest() });
    _palette.insert({ "CompareNums", BT::TreeNodeManifest() });
    _palette.insert({ "Info", BT::TreeNodeManifest() });

    AutonomyNodeIssueDetector nodeIssueDetector(node, NODEISSUDETECTOR_FILE, _factory, _palette);
    HealthError err = nodeIssueDetector.detect();
    printIssuesIf(nodeIssueDetector, nodeIssueDetector.issues().size() > 0);
    ASSERT_FALSE(err.error);
    std::vector<AutonomyIssue::Ptr> issues = nodeIssueDetector.issues();
    ASSERT_EQ(issues.size(), 0);
}


TEST_F(AutonomyNodeIssueDetectorTest, TestGoodCustomNodeWithPorts)
{
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement *node = walkTree(doc, NODEISSUDETECTOR_FILE,
        {
            {"BehaviorTree", 2},
            {"Sequence", 0},
            {"Fallback", 0},
            {"CallTriggerService", 0}
        });
    
    ASSERT_TRUE(node);

    //add our nodes to the model
    _palette.insert({ "CallSetBoolService", BT::TreeNodeManifest() });
    _palette.insert({ "CallTriggerService", BT::TreeNodeManifest() });
    _palette.insert({ "CompareNums", BT::TreeNodeManifest() });
    _palette.insert({ "Info", BT::TreeNodeManifest() });

    AutonomyNodeIssueDetector nodeIssueDetector(node, NODEISSUDETECTOR_FILE, _factory, _palette);
    HealthError err = nodeIssueDetector.detect();
    printIssuesIf(nodeIssueDetector, nodeIssueDetector.issues().size() > 0);
    ASSERT_FALSE(err.error);
    std::vector<AutonomyIssue::Ptr> issues = nodeIssueDetector.issues();
    ASSERT_EQ(issues.size(), 0);
}


TEST_F(AutonomyNodeIssueDetectorTest, TestNodeNotInManifest)
{
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement *node = walkTree(doc, BADMANIFEST_FILE,
        {
            {"BehaviorTree", 1},
            {"SomeNonexistentNode", 0}
        });
    
    ASSERT_TRUE(node);

    //add our node to the palette so we dont trip that issue (node detector does not do that for us)
    _palette.insert({ "SomeNonexistentNode", BT::TreeNodeManifest() });

    AutonomyNodeIssueDetector nodeIssueDetector(node, BADMANIFEST_FILE, _factory, _palette);
    HealthError err = nodeIssueDetector.detect();
    printIssuesIf(nodeIssueDetector, nodeIssueDetector.issues().size() == 0);
    ASSERT_TRUE(err.error);
    std::vector<AutonomyIssue::Ptr> issues = nodeIssueDetector.issues();
    ASSERT_EQ(issues.size(), 1);
    AutonomyIssue::Ptr iss = issues[0];
    ASSERT_EQ(iss->type(), "UndefinedIssue");
}


TEST_F(AutonomyNodeIssueDetectorTest, TestNodeNotInModel)
{
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement *node = walkTree(doc, BADMANIFEST_FILE,
        {
            {"BehaviorTree", 0},
            {"CallSetBoolService", 0}
        });
    
    ASSERT_TRUE(node);

    //add our node to the palette so we dont trip that issue (node detector does not do that for us)
    _palette.insert({ "SomeNonexistentNode", BT::TreeNodeManifest() });

    AutonomyNodeIssueDetector nodeIssueDetector(node, BADMANIFEST_FILE, _factory, _palette);
    HealthError err = nodeIssueDetector.detect();
    printIssuesIf(nodeIssueDetector, nodeIssueDetector.issues().size() == 0);
    ASSERT_TRUE(err.error);
    std::vector<AutonomyIssue::Ptr> issues = nodeIssueDetector.issues();
    ASSERT_EQ(issues.size(), 1);
    AutonomyIssue::Ptr iss = issues[0];
    ASSERT_EQ(iss->type(), "ModelError");
}

//tests that output ports are optional
TEST_F(AutonomyNodeIssueDetectorTest, TestGoodCustomNodeWithBlankOutputPort)
{
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement *node = walkTree(doc, NODEISSUDETECTOR_FILE,
        {
            {"BehaviorTree", 1},
            {"Sequence", 0},
            {"GetBoolTopic", 0}
        });
    
    ASSERT_TRUE(node);

    //add our nodes to the model
    _palette.insert({ "GetBoolTopic", BT::TreeNodeManifest() });
    _palette.insert({ "PublishUInt16", BT::TreeNodeManifest() });

    AutonomyNodeIssueDetector nodeIssueDetector(node, NODEISSUDETECTOR_FILE, _factory, _palette);
    HealthError err = nodeIssueDetector.detect();
    printIssuesIf(nodeIssueDetector, nodeIssueDetector.issues().size() > 0);
    ASSERT_FALSE(err.error);
    std::vector<AutonomyIssue::Ptr> issues = nodeIssueDetector.issues();
    ASSERT_EQ(issues.size(), 0);
}

TEST_F(AutonomyNodeIssueDetectorTest, TestNodeWithoutBracedOutput)
{
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement *node = walkTree(doc, NODEISSUDETECTOR_FILE,
        {
            {"BehaviorTree", 1},
            {"Sequence", 0},
            {"GetBoolTopic", 1}
        });
    
    ASSERT_TRUE(node);

    //add our nodes to the model
    _palette.insert({ "GetBoolTopic", BT::TreeNodeManifest() });
    _palette.insert({ "PublishUInt16", BT::TreeNodeManifest() });
    
    AutonomyNodeIssueDetector nodeIssueDetector(node, NODEISSUDETECTOR_FILE, _factory, _palette);
    HealthError err = nodeIssueDetector.detect();
    printIssuesIf(nodeIssueDetector, nodeIssueDetector.issues().size() == 0);
    ASSERT_FALSE(err.error);
    std::vector<AutonomyIssue::Ptr> issues = nodeIssueDetector.issues();
    ASSERT_EQ(issues.size(), 1);
    ASSERT_EQ(issues[0]->type(), "OutputPortFormatIssue");
}

TEST_F(AutonomyNodeIssueDetectorTest, TestGoodOutput)
{
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement *node = walkTree(doc, NODEISSUDETECTOR_FILE,
        {
            {"BehaviorTree", 1},
            {"Sequence", 0},
            {"GetBoolTopic", 2}
        });
    
    ASSERT_TRUE(node);

    //add our nodes to the model
    _palette.insert({ "GetBoolTopic", BT::TreeNodeManifest() });
    _palette.insert({ "PublishUInt16", BT::TreeNodeManifest() });
    
    AutonomyNodeIssueDetector nodeIssueDetector(node, NODEISSUDETECTOR_FILE, _factory, _palette);
    HealthError err = nodeIssueDetector.detect();
    printIssuesIf(nodeIssueDetector, nodeIssueDetector.issues().size() != 0);
    ASSERT_FALSE(err.error);
    std::vector<AutonomyIssue::Ptr> issues = nodeIssueDetector.issues();
    ASSERT_EQ(issues.size(), 0);
}

//tests that blank required inputs are found
TEST_F(AutonomyNodeIssueDetectorTest, TestCustomNodeWithBlankRequiredInputPort)
{
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement *node = walkTree(doc, NODEISSUDETECTOR_FILE,
        {
            {"BehaviorTree", 1},
            {"Sequence", 0},
            {"PublishUInt16", 0}
        });
    
    ASSERT_TRUE(node);

    //add our nodes to the model
    _palette.insert({ "GetBoolTopic", BT::TreeNodeManifest() });
    _palette.insert({ "PublishUInt16", BT::TreeNodeManifest() });
    
    AutonomyNodeIssueDetector nodeIssueDetector(node, NODEISSUDETECTOR_FILE, _factory, _palette);
    HealthError err = nodeIssueDetector.detect();
    printIssuesIf(nodeIssueDetector, nodeIssueDetector.issues().size() == 0);
    ASSERT_FALSE(err.error);
    std::vector<AutonomyIssue::Ptr> issues = nodeIssueDetector.issues();
    ASSERT_EQ(issues.size(), 1);
    AutonomyIssue::Ptr iss = issues[0];
    ASSERT_EQ(iss->type(), "RequiredPortError");
}

//tests that ports on builtin nodes are implicitly required
TEST_F(AutonomyNodeIssueDetectorTest, TestBuiltinNodeWithBlankInputPort)
{
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement *node = walkTree(doc, NODEISSUDETECTOR_FILE,
        {
            {"BehaviorTree", 1},
            {"Sequence", 0},
            {"SetBlackboard", 0}
        });
    
    ASSERT_TRUE(node);

    //add our nodes to the model
    _palette.insert({ "GetBoolTopic", BT::TreeNodeManifest() });
    _palette.insert({ "PublishUInt16", BT::TreeNodeManifest() });
    
    AutonomyNodeIssueDetector nodeIssueDetector(node, NODEISSUDETECTOR_FILE, _factory, _palette);
    HealthError err = nodeIssueDetector.detect();
    printIssuesIf(nodeIssueDetector, nodeIssueDetector.issues().size() == 0);
    ASSERT_FALSE(err.error);
    std::vector<AutonomyIssue::Ptr> issues = nodeIssueDetector.issues();
    ASSERT_EQ(issues.size(), 1);
    AutonomyIssue::Ptr iss = issues[0];
    ASSERT_EQ(iss->type(), "RequiredPortError");
}


TEST_F(AutonomyNodeIssueDetectorTest, TestBuiltinNodeWithBlankInOutPort)
{
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement *node = walkTree(doc, NODEISSUDETECTOR_FILE,
        {
            {"BehaviorTree", 1},
            {"Sequence", 0},
            {"SetBlackboard", 1}
        });
    
    ASSERT_TRUE(node);

    //add our nodes to the model
    _palette.insert({ "GetBoolTopic", BT::TreeNodeManifest() });
    _palette.insert({ "PublishUInt16", BT::TreeNodeManifest() });
    
    AutonomyNodeIssueDetector nodeIssueDetector(node, NODEISSUDETECTOR_FILE, _factory, _palette);
    HealthError err = nodeIssueDetector.detect();
    printIssuesIf(nodeIssueDetector, nodeIssueDetector.issues().size() == 0);
    ASSERT_FALSE(err.error);
    std::vector<AutonomyIssue::Ptr> issues = nodeIssueDetector.issues();
    ASSERT_EQ(issues.size(), 1);
    AutonomyIssue::Ptr iss = issues[0];
    ASSERT_EQ(iss->type(), "RequiredPortError");
}


TEST_F(AutonomyNodeIssueDetectorTest, TestBlankScript)
{
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement *node = walkTree(doc, NODEISSUDETECTOR_FILE,
        {
            {"BehaviorTree", 1},
            {"Sequence", 0},
            {"Script", 0}
        });
    
    ASSERT_TRUE(node);

    //add our nodes to the model
    _palette.insert({ "GetBoolTopic", BT::TreeNodeManifest() });
    _palette.insert({ "PublishUInt16", BT::TreeNodeManifest() });
    
    AutonomyNodeIssueDetector nodeIssueDetector(node, NODEISSUDETECTOR_FILE, _factory, _palette);
    HealthError err = nodeIssueDetector.detect();
    printIssuesIf(nodeIssueDetector, nodeIssueDetector.issues().size() == 0);
    ASSERT_FALSE(err.error);
    std::vector<AutonomyIssue::Ptr> issues = nodeIssueDetector.issues();
    ASSERT_EQ(issues.size(), 1);
    AutonomyIssue::Ptr iss = issues[0];
    ASSERT_EQ(iss->type(), "RequiredPortError");
}


TEST_F(AutonomyNodeIssueDetectorTest, TestGoodScript)
{
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement *node = walkTree(doc, NODEISSUDETECTOR_FILE,
        {
            {"BehaviorTree", 1},
            {"Sequence", 0},
            {"Script", 1}
        });
    
    ASSERT_TRUE(node);

    //add our nodes to the model
    _palette.insert({ "GetBoolTopic", BT::TreeNodeManifest() });
    _palette.insert({ "PublishUInt16", BT::TreeNodeManifest() });
    
    AutonomyNodeIssueDetector nodeIssueDetector(node, NODEISSUDETECTOR_FILE, _factory, _palette);
    HealthError err = nodeIssueDetector.detect();
    printIssuesIf(nodeIssueDetector, nodeIssueDetector.issues().size() != 0);
    ASSERT_FALSE(err.error);
    std::vector<AutonomyIssue::Ptr> issues = nodeIssueDetector.issues();
    ASSERT_EQ(issues.size(), 0);
}


TEST_F(AutonomyNodeIssueDetectorTest, TestGoodSetBlackboardNode)
{
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement *node = walkTree(doc, NODEISSUDETECTOR_FILE,
        {
            {"BehaviorTree", 1},
            {"Sequence", 0},
            {"SetBlackboard", 2}
        });
    
    ASSERT_TRUE(node);

    //add our nodes to the model
    _palette.insert({ "GetBoolTopic", BT::TreeNodeManifest() });
    _palette.insert({ "PublishUInt16", BT::TreeNodeManifest() });
    
    AutonomyNodeIssueDetector nodeIssueDetector(node, NODEISSUDETECTOR_FILE, _factory, _palette);
    HealthError err = nodeIssueDetector.detect();
    printIssuesIf(nodeIssueDetector, nodeIssueDetector.issues().size() != 0);
    ASSERT_FALSE(err.error);
    std::vector<AutonomyIssue::Ptr> issues = nodeIssueDetector.issues();
    ASSERT_EQ(issues.size(), 0);

    std::vector<std::string> bbDefs = nodeIssueDetector.blackboardDefinitions();
    ASSERT_EQ(bbDefs.size(), 1);
    ASSERT_EQ(bbDefs[0], "value");
}
