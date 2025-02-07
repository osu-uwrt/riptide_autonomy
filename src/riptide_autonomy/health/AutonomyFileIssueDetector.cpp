#include "riptide_autonomy/autonomy_health.hpp"

AutonomyFileIssueDetector::AutonomyFileIssueDetector(const std::string& file, const BT::BehaviorTreeFactory& factory)
 : _file(file),
   _factory(factory) { }


HealthError AutonomyFileIssueDetector::detect()
{
    std::string cwd = _file.substr(0, _file.rfind('/'));

    //use syncissuedetector to scan for code issues but also parse the palette
    auto syncIssueDetector = std::make_shared<AutonomySyncIssueDetector>(_file, _factory);
    addSubdetector(syncIssueDetector); //will also run detection process

    //now access the sync issue detector palette as our own
    _palette = syncIssueDetector->palette();

    tinyxml2::XMLDocument xmlDoc;
    xmlDoc.LoadFile(_file.c_str());
    if(xmlDoc.Error())
    {
        addIssue(std::make_shared<UnfixableAutonomyIssue>(ISSUE_ERROR, _file, 1, "XMLError", xmlDoc.ErrorStr()));
        return;
    }

    tinyxml2::XMLElement *rootElement = xmlDoc.RootElement()->FirstChildElement("root");
    if(!rootElement)
    {
        addIssue(
            std::make_shared<UnfixableAutonomyIssue>(
                ISSUE_ERROR, 
                _file,
                1,
                "XMLError",
                "Missing BT root node"));
        
        return HealthError(true, "Stopped detection due to fatal errors");
    }

    auto treeDetector = std::make_shared<AutonomyTreeIssueDetector>(_file, cwd, rootElement, _factory, _palette);
    addSubdetector(treeDetector); //function will run detector

    //now make the palette accessible through our accessor
    _palette = treeDetector->palette();
}


NodeManifests AutonomyFileIssueDetector::palette() const
{
    return _palette;
}
