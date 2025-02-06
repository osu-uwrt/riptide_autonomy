#include "riptide_autonomy/autonomy_health.hpp"

std::vector<AutonomyIssue::Ptr> AutonomyIssueDetector::issues() const
{
    return _issues;
}


void AutonomyIssueDetector::addIssue(const AutonomyIssue::Ptr& issue)
{
    _issues.push_back(issue);
}


void AutonomyIssueDetector::addSubdetector(const AutonomyIssueDetector::Ptr& detector)
{
    _subdetectors.push_back(detector);
}
