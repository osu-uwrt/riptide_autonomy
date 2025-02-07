#include "riptide_autonomy/autonomy_health.hpp"


AutonomyTreeIssueDetector::AutonomyTreeIssueDetector(
    const std::string& fileName,
    const std::string& cwd,
    tinyxml2::XMLElement *root,
    const BT::BehaviorTreeFactory& factory,
    const NodeManifests& palette)
 : _fileName(fileName),
   _cwd(cwd),
   _rootElement(root),
   _factory(factory),
   _palette(palette) { }

HealthError AutonomyTreeIssueDetector::detect()
{
    //process includes first to ensure that subtrees will be recognized
    for(
        tinyxml2::XMLElement *includeElement = _rootElement->FirstChildElement("include");
        includeElement;
        includeElement = includeElement->NextSiblingElement("include"))
    {
        const char *pathAttribute = includeElement->Attribute("path");
        if(!pathAttribute)
        {
            addIssue(
                std::make_shared<UnfixableAutonomyIssue>(
                    ISSUE_ERROR,
                    _fileName,
                    includeElement->GetLineNum(),
                    "UnspecifiedInclude",
                    "Include tag does not specify a path"));
            
            continue;
        }

        std::shared_ptr<AutonomyFileIssueDetector> fileDetector = 
            std::make_shared<AutonomyFileIssueDetector>(
                _cwd + _fileName, _factory);

        addSubdetector(fileDetector);
    }

    //now process trees. Assume our palette is correct
    for(
        tinyxml2::XMLElement *behaviorTree = _rootElement->FirstChildElement("BehaviorTree");
        behaviorTree;
        behaviorTree = behaviorTree->NextSiblingElement("BehaviorTree"))
    {
        std::vector<std::string> bbDefs;
        tinyxml2::XMLElement *treeRoot = behaviorTree->FirstChildElement();

        if(!treeRoot)
        {
            addIssue(
                std::make_shared<UnfixableAutonomyIssue>(
                    ISSUE_WARN,
                    _fileName,
                    behaviorTree->GetLineNum(),
                    "EmptyTree",
                    "Behavior Tree is empty"));
            
            continue;
        }

        processTreeRecursive(treeRoot, bbDefs);
    }
}


NodeManifests AutonomyTreeIssueDetector::palette() const
{
    return _palette;
}

// addSubdetector override to ensure that palette and blackboard can be updated by subdetectors.
void AutonomyTreeIssueDetector::addSubdetector(const AutonomyIssueDetector::Ptr& detector)
{
    if(auto treeDetector = std::dynamic_pointer_cast<AutonomyTreeIssueDetector>(detector))
    {
        //add detector the normal way (also runs detection)
        AutonomyIssueDetector::addSubdetector(detector);

        //...now pull palette and blackboard out of detector
        mergeNewPalette(treeDetector->palette());
    }
    else if(auto fileDetector = std::dynamic_pointer_cast<AutonomyFileIssueDetector>(detector))
    {
        AutonomyIssueDetector::addSubdetector(detector);

        //now pull palette out of detector
        mergeNewPalette(treeDetector->palette());
    }
}


void AutonomyTreeIssueDetector::processTreeRecursive(tinyxml2::XMLElement *treeRoot, std::vector<std::string>& blackboardDefinitions)
{
    // spawn and run a node issue detector for the tree root first
    auto nodeIssueDetector = std::make_shared<AutonomyNodeIssueDetector>(treeRoot, _factory, _palette, blackboardDefinitions);
    addSubdetector(nodeIssueDetector);

    // now add outputs to blackboard...
    

    // ensure that if there are children, the node is a decorator or control

    // ensure that if there are multiple children, the node is a control

    // now recursively call this function on all children
}


void AutonomyTreeIssueDetector::mergeNewPalette(const NodeManifests& palette)
{

}
