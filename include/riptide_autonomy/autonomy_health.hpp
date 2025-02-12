#pragma once

#include "riptide_autonomy/autonomy_base.hpp"
#include <tinyxml2.h>

typedef std::unordered_map<std::string, BT::TreeNodeManifest> NodeManifests;

struct HealthError
{
    HealthError(bool error, const std::string message)
     : error(error),
       message(message) { }

    bool error;
    std::string message;
};


enum AutonomyIssueSeverity
{
    ISSUE_INFO,
    ISSUE_WARN,
    ISSUE_ERROR
};


class AutonomyIssue
{
    public:
    typedef std::shared_ptr<AutonomyIssue> Ptr;

    static std::string fileAndLine(const std::string& file, const tinyxml2::XMLElement *element);

    AutonomyIssue(
        const AutonomyIssueSeverity& severity,
        const std::string& file,
        int line,
        const std::string& type,
        const std::string& description);

    AutonomyIssueSeverity severity() const;
    std::string file() const;
    int line() const;
    std::string type() const;
    std::string issue() const;

    virtual HealthError fix() = 0;

    private:
    const AutonomyIssueSeverity _severity;
    const int _line;
    const std::string 
        _file,
        _type,
        _description;
};


class UnfixableAutonomyIssue : public AutonomyIssue
{
    public:
    UnfixableAutonomyIssue(
        const AutonomyIssueSeverity& severity,
        const std::string& file,
        int line,
        const std::string& type,
        const std::string& description)
     : AutonomyIssue(severity, file, line, type, description) { }

    HealthError fix()
    {
        return HealthError(true, "Issue cannot be automatically fixed.");
    }
};


class AutonomyIssueDetector
{
    public:
    typedef std::shared_ptr<AutonomyIssueDetector> Ptr;

    virtual HealthError detect() = 0;
    std::vector<AutonomyIssue::Ptr> issues() const;

    protected:
    void addIssue(const AutonomyIssue::Ptr& issue);
    void addSubdetector(const AutonomyIssueDetector::Ptr& detector);

    private:
    std::vector<AutonomyIssue::Ptr> _issues;
    std::vector<AutonomyIssueDetector::Ptr> _subdetectors;
};


/**
 * Detects runtime issues for the entire autonomy system, including sync, all trees, and XML nodes.
 */
class AutonomySystemIssueDetector : public AutonomyIssueDetector
{
    public:
    HealthError detect() override;
};

//defined in AutonomySyncIssueDetector.cpp
class AutonomyNodeMismatchIssue : public AutonomyIssue
{
    public:
    AutonomyNodeMismatchIssue(
        const std::string& file, 
        int line, 
        const std::string& nodeId, 
        bool fixableInXml,
        const std::string& description);

    HealthError fix() override;

    private:
    const std::string _nodeId;
    const bool _fixableInXml;
};

/**
 * Detects issues in information sync between code and XML
 */
class AutonomySyncIssueDetector : public AutonomyIssueDetector
{
    public:
    static std::string portDirectionToString(const BT::PortDirection& direction);
    static BT::PortDirection stringToPortDirection(const std::string& str);

    AutonomySyncIssueDetector(
        const std::string& file,
        const BT::BehaviorTreeFactory& factory);

    HealthError detect() override;
    NodeManifests palette() const;

    private:
    tinyxml2::XMLElement *detectTreeNodesModel(tinyxml2::XMLDocument& xmlDoc);
    bool detectIdAndTypeIssues(const char *xmlId, const char *xmlType, tinyxml2::XMLElement *nodeElement);
    bool detectPortIssues(const char *xmlId, tinyxml2::XMLElement* nodeElement);

    const std::string _file;
    const BT::BehaviorTreeFactory& _factory;

    NodeManifests _palette;
};


class AutonomyFileIssueDetector : public AutonomyIssueDetector
{
    public:
    AutonomyFileIssueDetector(const std::string& file, const BT::BehaviorTreeFactory& factory);
    HealthError detect() override;
    NodeManifests palette() const;
    std::string file() const;

    private:
    const std::string _file;
    const BT::BehaviorTreeFactory& _factory;
    NodeManifests _palette;
};


class AutonomyOmittedIssue : public AutonomyIssue
{
    public:
    AutonomyOmittedIssue(const std::string& file);
    HealthError fix();

    private:
    std::string file;
};

//
// This stuff defines how we analyze control nodes to detect potentially undefined blackboard entries
//

typedef std::function<std::vector<int>(size_t n)> NodeExecutionOrder;

enum NodeExecutionBlackboardLinkStatus
{
    BLACKBOARD_LINKED,
    BLACKBOARD_UNLINKED
};

struct NodeExecutionOrderWithBlackboard
{
    NodeExecutionOrderWithBlackboard(const NodeExecutionOrder& order, NodeExecutionBlackboardLinkStatus blackboardLinked)
     : order(order),
       blackboardLinked(blackboardLinked) { }

    NodeExecutionOrder order;
    NodeExecutionBlackboardLinkStatus blackboardLinked;
};

typedef std::vector<NodeExecutionOrderWithBlackboard> NodeExecutionDescription;
static const std::map<std::string, NodeExecutionDescription> NODE_EXECUTION_DESCRIPTIONS();

/**
 * Detects issues in specific trees such as bad includes, overpopulated decorators, etc.
 * Invokes AutonomyNodeIssueDetector as a subdetector.
 */
class AutonomyTreeIssueDetector : public AutonomyIssueDetector
{
    public:
    AutonomyTreeIssueDetector(
        const std::string& fileName,
        const std::string& cwd,
        tinyxml2::XMLElement *root,
        const BT::BehaviorTreeFactory& factory,
        const NodeManifests& palette);

    HealthError detect() override;
    NodeManifests palette() const;
    std::string file() const;

    protected:
    void addSubdetector(const AutonomyIssueDetector::Ptr& detector);
    
    private:
    void processTreeRecursive(tinyxml2::XMLElement *treeRoot, std::vector<std::string>& blackboardDefinitions);
    void mergeNewPalette(const NodeManifests& palette);

    const std::string _fileName, _cwd;
    const BT::BehaviorTreeFactory& _factory;
    NodeManifests _palette;
    
    tinyxml2::XMLElement *_rootElement;
};


class AutonomyUndefinedIssue : public AutonomyIssue
{
    public:
    AutonomyUndefinedIssue(tinyxml2::XMLElement *node);
    HealthError fix();
};


/**
 * Detects issues in specific XML node instances, like unfilled required ports
 */
class AutonomyNodeIssueDetector : public AutonomyIssueDetector
{
    public:
    AutonomyNodeIssueDetector(
        tinyxml2::XMLElement *node,
        const std::string& file,
        const BT::BehaviorTreeFactory& factory,
        const NodeManifests& palette,
        const std::vector<std::string>& blackboardDefinitions = {});
    
    HealthError detect() override;

    private:
    tinyxml2::XMLElement *_node;
    const std::string _fileName;
    const BT::BehaviorTreeFactory& _factory;
    NodeManifests _palette;
    std::vector<std::string> _blackboardDefs;
};
