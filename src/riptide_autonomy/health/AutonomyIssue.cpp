#include "riptide_autonomy/autonomy_health.hpp"


std::string AutonomyIssue::fileAndLine(const std::string& file, const tinyxml2::XMLElement *element)
{
    return file + ": " + std::to_string(element->GetLineNum());
}


AutonomyIssue::AutonomyIssue(
    const AutonomyIssueSeverity& severity,
    const std::string& file,
    int line,
    const std::string& type,
    const std::string& description)
 : _severity(severity),
   _file(file),
   _line(line),
   _type(type),
   _description(description) { }


AutonomyIssueSeverity AutonomyIssue::severity() const
{
    return _severity;
}


std::string AutonomyIssue::file() const
{
    return _file;
}


int AutonomyIssue::line() const
{
    return _line;
}


std::string AutonomyIssue::type() const
{
    return _type;
}


std::string AutonomyIssue::issue() const
{
    return _description;
}
