#pragma once

#include "riptide_autonomy/autonomy_base.hpp"


typedef std::unordered_map<std::string, BT::TreeNodeManifest> NodeManifests;

struct HealthError
{
    HealthError()
     : error(false),
       message("") { }

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

    AutonomyIssue(const AutonomyIssueSeverity& severity, const std::string& type, const std::string& description);

    AutonomyIssueSeverity severity();
    std::string type();
    std::string issue();

    virtual HealthError fix() = 0;

    private:
    const AutonomyIssueSeverity _severity;
    const std::string 
        _type,
        _description;
};


class AutonomyIssueDetector
{
    public:
    typedef std::shared_ptr<AutonomyIssueDetector> Ptr;

    virtual HealthError detect() = 0;
    std::vector<AutonomyIssue::Ptr> issues();

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
    
};

/**
 * Detects issues in information sync between code and XML
 */
class AutonomySyncIssueDetector : public AutonomyIssueDetector
{

};

/**
 * Detects issues in specific trees such as nonexistent bb variables or overpopulating decorators, etc
 */
class AutonomyTreeIssueDetector : public AutonomyIssueDetector
{
    
};

/**
 * Detects issues in specific XML node instances, like unfilled required ports
 */
class AutonomyNodeIssueDetector : public AutonomyIssueDetector
{

};
