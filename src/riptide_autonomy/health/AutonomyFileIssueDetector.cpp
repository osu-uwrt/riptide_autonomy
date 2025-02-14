#include "riptide_autonomy/autonomy_health.hpp"

AutonomyFileIssueDetector::AutonomyFileIssueDetector(const std::string& file, std::shared_ptr<const BT::BehaviorTreeFactory> factory)
 : _file(file),
   _factory(factory) { }


HealthError AutonomyFileIssueDetector::detect()
{
    std::string cwd = _file.substr(0, _file.rfind('/'));

    //open the autonomy project and make sure this file is included
    tinyxml2::XMLDocument projDoc;
    projDoc.LoadFile(AUTONOMY_BTPROJ.c_str());
    if(projDoc.Error())
    {
        addIssue(
            std::make_shared<UnfixableAutonomyIssue>(
                ISSUE_ERROR, 
                _file, 
                1, 
                "ProjectError", 
                ("While loading autonomy project: " + std::string(projDoc.ErrorStr())).c_str()));
    }

    tinyxml2::XMLElement *projRootElement = projDoc.RootElement()->FirstChildElement("root");
    if(!projRootElement)
    {
        addIssue(
            std::make_shared<UnfixableAutonomyIssue>(
                ISSUE_ERROR,
                AUTONOMY_BTPROJ,
                1,
                "XMLError",
                "Project missing BT root node"));
        
        return HealthError(true, "Aborted due to earlier issues");
    }


    bool hasFileInProject = false;
    for(
        tinyxml2::XMLElement *includeTag = projRootElement->FirstChildElement("include");
        includeTag;
        includeTag = includeTag->NextSiblingElement("include"))
    {
        const char *path = includeTag->Attribute("path");
        if(!path)
        {
            addIssue(
                std::make_shared<UnfixableAutonomyIssue>(
                    ISSUE_ERROR,
                    AUTONOMY_BTPROJ,
                    includeTag->GetLineNum(),
                    "XMLError",
                    "Include tag missing path"));
            
            return HealthError(true, "Aborted due to earlier issues");
        }

        if(_file == path)
        {
            hasFileInProject = true;
            break;
        }
    }

    if(!hasFileInProject)
    {
        addIssue(std::make_shared<AutonomyOmittedIssue>(_file));
        return HealthError(true, "Aborted due to earlier issues");
    }

    //use syncissuedetector to scan for code issues but also parse the palette
    auto syncIssueDetector = std::make_shared<AutonomySyncIssueDetector>(_file, _factory);
    addSubdetector(syncIssueDetector); //will also run detection process

    //now access the sync issue detector palette as our own
    _palette = syncIssueDetector->palette();

    tinyxml2::XMLDocument fileXmlDoc;
    fileXmlDoc.LoadFile(_file.c_str());
    if(fileXmlDoc.Error())
    {
        addIssue(std::make_shared<UnfixableAutonomyIssue>(ISSUE_ERROR, _file, 1, "XMLError", fileXmlDoc.ErrorStr()));
        return HealthError(true, "Aborted due to earlier issues");
    }

    tinyxml2::XMLElement *fileRootElement = fileXmlDoc.RootElement()->FirstChildElement("root");
    if(!fileRootElement)
    {
        addIssue(
            std::make_shared<UnfixableAutonomyIssue>(
                ISSUE_ERROR, 
                _file,
                1,
                "XMLError",
                "Missing BT root node"));
        
        return HealthError(true, "Aborted due to earlier issues");
    }

    auto treeDetector = std::make_shared<AutonomyTreeIssueDetector>(_file, cwd, fileRootElement, _factory, _palette);
    addSubdetector(treeDetector); //function will run detector

    //now make the palette accessible through our accessor
    _palette = treeDetector->palette();
}


NodeManifests AutonomyFileIssueDetector::palette() const
{
    return _palette;
}


std::string AutonomyFileIssueDetector::file() const
{
    return _file;
}
