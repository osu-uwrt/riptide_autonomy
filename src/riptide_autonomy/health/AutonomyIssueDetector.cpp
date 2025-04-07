#include "riptide_autonomy/autonomy_health.hpp"

std::vector<AutonomyIssue::Ptr> AutonomyIssueDetector::issues() const
{
    std::vector<AutonomyIssue::Ptr> v(_issues);

    //add issues from all subdetectors now
    for(AutonomyIssueDetector::Ptr subdetector : _subdetectors)
    {
        std::vector<AutonomyIssue::Ptr> subIssues = subdetector->issues();
        v.insert(v.end(), subIssues.begin(), subIssues.end());
    }

    return v;
}


void AutonomyIssueDetector::addIssue(const AutonomyIssue::Ptr& issue)
{
    _issues.push_back(issue);
}


void AutonomyIssueDetector::addSubdetector(const AutonomyIssueDetector::Ptr& detector)
{
    _subdetectors.push_back(detector);
    detector->detect();
}
