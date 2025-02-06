#include "riptide_autonomy/autonomy_health.hpp"


AutonomyIssue::AutonomyIssue(const AutonomyIssueSeverity& severity, const std::string& type, const std::string& description)
 : _severity(severity),
   _type(type),
   _description(description) { }


AutonomyIssueSeverity AutonomyIssue::severity() const
{
    return _severity;
}


std::string AutonomyIssue::type() const
{
    return _type;
}


std::string AutonomyIssue::issue() const
{
    return _description;
}
