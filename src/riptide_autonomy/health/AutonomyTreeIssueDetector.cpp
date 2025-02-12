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


std::string AutonomyTreeIssueDetector::file() const
{
    return _fileName;
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
        mergeNewPalette(fileDetector->palette());
    }
}


void AutonomyTreeIssueDetector::processTreeRecursive(tinyxml2::XMLElement *treeRoot, std::vector<std::string>& blackboardDefinitions)
{
    std::string nodeName = treeRoot->Name();

    // spawn and run a node issue detector for the tree root first
    auto nodeIssueDetector = std::make_shared<AutonomyNodeIssueDetector>(treeRoot, _fileName, _factory, _palette, blackboardDefinitions);
    addSubdetector(nodeIssueDetector);

    if(_palette.count(nodeName) == 0)
    {
        //cant do any of the rest of the tests without knowing what the node is.
        //node subdetector should have already caught and reported this so we wont here.
        return;
    }

    // now add outputs to blackboard...
    for(const tinyxml2::XMLAttribute *nodeAttribute = treeRoot->FirstAttribute();
        nodeAttribute;
        nodeAttribute = nodeAttribute->Next())
    {
        const char 
            *name = nodeAttribute->Name(),
            *value = nodeAttribute->Value();
        
        //check that port is in the palette. palette contains node because we already checked that
        //If it is not we will also ignore it
        BT::PortsList ports = _palette.at(nodeName).ports;
        if( name && value
            && ports.count(name) > 0 
            && ports.at(name).direction() == BT::PortDirection::OUTPUT)
        {
            //if we get here, value exists and this is an output port. Add value to blackboard
            blackboardDefinitions.push_back(value);
        }
    }

    // put children into vector. Not only does this count them but it also helps us with exec order later
    std::vector<tinyxml2::XMLElement *> children;
    for(
        tinyxml2::XMLElement *child = treeRoot->FirstChildElement();
        child;
        child = child->NextSiblingElement())
    {
        children.push_back(child);
    }

    // ensure that if there are children, the node is a decorator or control. 
    // if there are multiple children, the node must be a control
    BT::NodeType nodeType = _palette.at(nodeName).type;
    
    if(nodeType == BT::NodeType::CONTROL && children.size() == 0)
    {
        addIssue(
            std::make_shared<UnfixableAutonomyIssue>(
                ISSUE_ERROR,
                _fileName,
                treeRoot->GetLineNum(),
                "BTError",
                "Control node cannot have zero children"));
        
        return;
    }

    if(nodeType == BT::NodeType::DECORATOR && children.size() != 1)
    {
        addIssue(
            std::make_shared<UnfixableAutonomyIssue>(
                ISSUE_ERROR,
                _fileName,
                treeRoot->GetLineNum(),
                "BTError",
                "Decorator must have exactly one child"));
        
        return;
    }

    if((nodeType == BT::NodeType::ACTION
        || nodeType == BT::NodeType::DECORATOR
        || nodeType == BT::NodeType::SUBTREE)
        && children.size() > 0)
    {
        addIssue(
            std::make_shared<UnfixableAutonomyIssue>(
                ISSUE_ERROR,
                _fileName,
                treeRoot->GetLineNum(),
                "BTError",
                "Leaf nodes cannot have children."));
        
        return;
    }

    // now recursively call this function on all children, minding execution order
    
    //
    // DECO NODE ANALYSIS (EASY)
    //
    if(nodeType == BT::NodeType::DECORATOR)
    {
        processTreeRecursive(treeRoot->FirstChildElement(), blackboardDefinitions);
    } 
    
    //
    // CONTROL NODE ANALYSIS (HARD)
    //
    else if(nodeType == BT::NodeType::CONTROL)
    {
        NodeExecutionDescription execDesc = NODE_EXECUTION_DESCRIPTIONS().at("Sequence"); //default, executes L2R, blackboard linked

        // try to get execution description
        if(NODE_EXECUTION_DESCRIPTIONS().count(nodeName) == 0)
        {
            addIssue(
                std::make_shared<UnfixableAutonomyIssue>(
                    ISSUE_WARN,
                    _fileName,
                    treeRoot->GetLineNum(),
                    "BTWarning",
                    "Control node type " + std::string(nodeName) + " is not recognized by the system. "
                    "Execution order description will default to that of the Sequence node. Because of "
                    "this, blackboard variable availability detection may be inaccurate. To fix this, "
                    "program your execution order into the NodeExecutionDescriptions.cpp file."));
        } else
        {
            execDesc = NODE_EXECUTION_DESCRIPTIONS().at(nodeName);
        }

        for(size_t i = 0; i < execDesc.size(); i++)
        {
            NodeExecutionOrderWithBlackboard subDescription = execDesc[i];
            std::vector<int> order = subDescription.order(children.size());

            //quickly check that order will not try to tick a nonexistent child
            for(int idx : order)
            {
                if(idx >= children.size())
                {
                    addIssue(
                    std::make_shared<UnfixableAutonomyIssue>(
                        ISSUE_WARN,
                        _fileName,
                        treeRoot->GetLineNum(),
                        "BTWarning",
                        "INTERNAL ERROR: Node execution description for node " + std::string(nodeName) + " included"
                        "an index for a child node (" + std::to_string(idx) + " that does not exist."
                        "(we have " + std::to_string(children.size()) + ")"));
                }
            }

            if(subDescription.blackboardLinked == BLACKBOARD_LINKED)
            {
                //easier option. just give the same blackboard to all nodes sequentially
                for(int idx : order)
                {
                    processTreeRecursive(children[idx], blackboardDefinitions);
                }
            }
            else
            {
                //harder option. Need to analyze each subtree separately, then take the intersection of the blackboards.
                //the intersection are the blackboard variables that are guaranteed to exist after execution.

                std::vector<std::vector<std::string>> blackboardPossibilities;

                for(int idx : order)
                {
                    std::vector<std::string> scopedBlackboardDefs(blackboardDefinitions);
                    processTreeRecursive(children[idx], blackboardDefinitions);
                }

                //vector intersection: https://stackoverflow.com/questions/19483663/vector-intersection-in-c
                for(size_t j = 0; j < blackboardPossibilities.size(); j++)
                {
                    std::sort(blackboardPossibilities[j].begin(), blackboardPossibilities[j].end());
                }

                std::vector<std::string> newBlackboardDefs = blackboardPossibilities[0];
                for(size_t j = 1; j < blackboardPossibilities.size(); j++)
                {
                    std::vector<std::string> intersected;
                    std::set_intersection(newBlackboardDefs.begin(), newBlackboardDefs.end(),
                                          blackboardPossibilities[j].begin(), blackboardPossibilities[j].end(),
                                          std::back_inserter(intersected));
                    
                    newBlackboardDefs = intersected;
                }

                blackboardDefinitions = newBlackboardDefs; //now contains guaranteed blackboard defs
            }
        }
    }
}


void AutonomyTreeIssueDetector::mergeNewPalette(const NodeManifests& newPalette)
{
    //iterate through palette
    for(auto pair : newPalette)
    {        
        std::string nodeName = pair.first;
        BT::TreeNodeManifest newManifest = pair.second;

        if(_palette.count(nodeName) == 0)
        {
            _palette.insert(pair);
        }
    }
}
