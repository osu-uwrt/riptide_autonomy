#include <filesystem>

#include "riptide_autonomy/autonomy_lib.hpp"
#include "tinyxml2.h"

#ifndef AUTONOMY_PKG_NAME
#define AUTONOMY_PKG_NAME "riptide_autonomy2"
#endif

#define AUTONOMY_WORKSPACE "/osu-uwrt/development/software/src/riptide_autonomy/trees/.groot/workspace.xml"
#define AUTONOMY_ACTIONS "/osu-uwrt/development/software/src/riptide_autonomy/include/riptide_autonomy/bt_actions"
#define AUTONOMY_CONDITIONS "/osu-uwrt/development/software/src/riptide_autonomy/include/riptide_autonomy/bt_conditions"
#define AUTONOMY_DECORATORS "/osu-uwrt/development/software/src/riptide_autonomy/include/riptide_autonomy/bt_decorators"

using namespace tinyxml2;
using namespace BT;

bool yta = false;

const char *nodeId(XMLElement *node) {
    if(node->Attribute("ID")) {
        return node->Attribute("ID");
    }

    return node->Name();
}

XMLElement *findFirstChildByName(XMLElement *parent, const char *childName) {
    XMLElement *child = parent->FirstChildElement();
    while(child != nullptr) {
        if(strcmp(child->Value(), childName) == 0) {
            return child;
        }

        child = child->NextSiblingElement();
    }

    return child;
}

std::string portDirectionToString(PortDirection direction) {
    switch(direction) {
        case PortDirection::INPUT:
            return "input_port";
        case PortDirection::OUTPUT:
            return "output_port";
        case PortDirection::INOUT:
            return "inout_port";
    }

    return "";
}

/**
 * @brief Goes through Include to get all custom nodes
 * @return a list of the names of every custom node
 */
std::list<std::string> getCustomNodes(){
    const std::string HOME = getEnvVar("HOME");
    std::string actionsLoc = HOME + std::string(AUTONOMY_ACTIONS);
    std::string conditionsLoc = HOME + std::string(AUTONOMY_CONDITIONS);
    std::string decoratorsLoc = HOME + std::string(AUTONOMY_DECORATORS);
    std::list<std::string> nodes;
    for (const auto & entry : std::filesystem::directory_iterator(actionsLoc)){
        //remove everything but file name
        std::string path = entry.path();
        path.erase(0,actionsLoc.length()+1);
        int dot = path.find(".");
        path.erase(dot,path.length());
        nodes.push_back(path);
    }
    for (const auto & entry : std::filesystem::directory_iterator(conditionsLoc)){
        //remove everything but file name
        std::string path = entry.path();
        path.erase(0,conditionsLoc.length()+1);
        int dot = path.find(".");
        path.erase(dot,path.length());
        nodes.push_back(path);
    }
    for (const auto & entry : std::filesystem::directory_iterator(decoratorsLoc)){
        //remove everything but file name
        std::string path = entry.path();
        path.erase(0,decoratorsLoc.length()+1);
        int dot = path.find(".");
        path.erase(dot,path.length());
        nodes.push_back(path);
    }
    return nodes;
}

/**
 * @brief Creates and adds XML element contining port information to the groot workspae
 * 
 * @param missingPort the name of the port to add
 * @param info  all relevent information for each port
 * @param node the nmane of the node the port belongs to
 * @param doc filepath to groot workspace 
 */
void addPortToWorkspace(std::string missingPort, PortInfo info, XMLElement *node){
    if(info.direction() == BT::PortDirection::INPUT){
    node->InsertNewChildElement("input_port");
    }
    else{
        node->InsertNewChildElement("output_port");
    }
    auto newPort = node->LastChildElement();
    //if there is a default val, add it
    if(info.defaultValue().size() > 0){
        newPort->SetAttribute("default", info.defaultValue().c_str());
    }
    //set the name
    newPort->SetAttribute("name",missingPort.c_str());
    newPort->SetAttribute("required","false");
    newPort->SetText(info.description().c_str());   
}


bool checkPort(PortInfo src_port, XMLElement *port){
    std::string type = port->Value();
    if(type.compare("inout_port") == 0){
        return true;
    }
    else if(type.compare("output_port") == 0){
        if(src_port.direction() == BT::PortDirection::OUTPUT){
            return true;
        }
        RCLCPP_WARN(log, "%s is listed as an output port in groot but was not implemented as such.", port->Attribute("name"));
    }
    else{
        if(src_port.direction() == BT::PortDirection::INPUT){
            return true;
        }
        RCLCPP_WARN(log, "%s is listed as an input port in groot but was not implemented as such.", port->Attribute("name"));
    }                    

    return false;
}


bool checkPorts(XMLElement *node, PortsList src_ports){
    bool port_match = true;
    auto port = node->FirstChildElement();
    std::map<std::string, PortDirection> ports_found;
    while(port != nullptr){
        const char *portVal = port->Value();
        PortDirection grootPortDirection;
        if(strcmp(portVal, "input_port") == 0) {
            grootPortDirection = PortDirection::INPUT;
        } else if(strcmp(portVal, "output_port") == 0) {
            grootPortDirection = PortDirection::OUTPUT;
        } else if(strcmp(portVal, "inout_port") == 0) {
            grootPortDirection = PortDirection::INOUT;
        } else {
            RCLCPP_ERROR(log, "%s is not a valid port type", portVal);
            return false;
        }

        if(const char *portName = port->Attribute("name")) {
            //found a port with a name
            bool portFoundInCpp = src_ports.find(portName) != src_ports.end();
            bool portTypesMatch = (portFoundInCpp ? src_ports.at(portName).direction() == grootPortDirection : false);
            
            ports_found.insert({portName, grootPortDirection});

            if(!(portFoundInCpp && portTypesMatch)) {
                //port of correct name and type not found in c++ implementation
                RCLCPP_WARN(log, "%s Port %s in groot was not found in c++ implementation. Would you like to remove it? (Y/n): ", portVal, port->Attribute("name"));
                std::string ans;
                std::cin>>ans;

                if(std::tolower(ans[0]) == 'y' || yta) {
                    XMLElement *tmp = port;
                    port = port->NextSiblingElement();
                    tmp->Parent()->DeleteChild(tmp);

                    continue; //already advanced port to next value, just continue loop
                }
                else{
                    port_match = false;
                }
            }
        }

        port = port->NextSiblingElement();
    }

    // check the other way. make sure every port in the manifest is in groot
    if(src_ports.size() != ports_found.size()){
        for(auto srcPair : src_ports) {
            if(ports_found.find(srcPair.first) == ports_found.end()) {
                RCLCPP_WARN(log, "Port %s was not found in groot workspace for node %s. Would you like to add it? (Y/n)", srcPair.first.c_str(), nodeId(node));
                std::string ans;
                std::cin>>ans;

                if(std::tolower(ans[0]) == 'y' || yta){
                    addPortToWorkspace(srcPair.first, srcPair.second, node);
                }
                else{
                    port_match = false;
                }
                
            } else {
                //port exists, check direction
                //TODO: this not fully working yet
                if(srcPair.second.direction() != ports_found[srcPair.first]) {
                    std::string 
                        srcDirectionStr = portDirectionToString(srcPair.second.direction()),
                        grootDirectionStr = portDirectionToString(ports_found[srcPair.first]),
                        ans;

                    RCLCPP_WARN(log,
                        "Port %s for node %s listed as an %s port in the c++ implementation, but as a %s port in Groot. Would you like to change the Groot type to %s (Y/n)?", 
                        srcPair.first.c_str(),
                        nodeId(node),
                        srcDirectionStr.c_str(),
                        grootDirectionStr.c_str(),
                        srcDirectionStr.c_str()
                    );

                    std::cin >> ans;
                    if(std::tolower(ans[0]) == 'y' || yta) {
                        //change the value of the port to srcDirectionStr
                        //need to find the node first
                        XMLElement *p = node->FirstChildElement();
                        while(p != nullptr) {
                            if(const char *pName = p->Attribute("name")) {
                                if(strcmp(pName, srcPair.first.c_str()) == 0) {
                                    p->SetValue(srcDirectionStr.c_str());
                                    break;
                                }
                            }
                            p = p->NextSiblingElement();
                        }
                    } else {
                        port_match = false;
                    }
                }
            }
        }
    }

    return port_match;
}

/**
 * @brief Checks to see if a certain XML element is found in riptide_autonomy 
 * 
 * @param test The XML node in the groot workspace
 * @param factory The BT factory that contains the riptide_autonomy implementations
 * @return true if the XML node exactly matches the riptide_autonomy implementation
 * @return false if there are discrepencies between the groot workspace and riptide_autonomy
 */
std::pair<bool,bool> findNodeInManifest(XMLElement *test, std::shared_ptr<BehaviorTreeFactory> factory){
    bool port_error = true;
    bool node_error = true;
    //arbitrarily return true for subtrees we only care about conditions/actions/decorators
    std::string testNodeType = test->Name();
    
    RCLCPP_INFO(log,"Checking Node %s", nodeId(test));
    //get the manifest
    std::unordered_map<std::string, BT::TreeNodeManifest> src_manifest = factory->manifests();    
    // try to find node in manifest
    try{
        BT::TreeNodeManifest node_maifest = src_manifest.at(nodeId(test));
        node_error = false;
        // node exists! check to make sure ports match. checkPorts returns true if the ports match, so false means error
        port_error = !checkPorts(test, node_maifest.ports);
    }
    catch(...){ }

    return {node_error, port_error};
}

/**
 * @brief Creates and adds XML element contining node information to the groot workspae
 * 
 * @param name the name of the node to add
 * @param ports the assocaited ports list contianing all ports and all relevent information for each port
 * @param type the node type (Action, Condition, Decorator)
 */
void addNodeToWorkspace(XMLDocument& tree, const std::string& name, PortsList ports, BT::NodeType type){
    //load Groot workspace
    auto model = tree.RootElement()->LastChildElement("TreeNodesModel");
    //add the node to the end of the TreeNodesModel
    if(type == BT::NodeType::ACTION){
        model->InsertNewChildElement("Action")->SetAttribute("ID",name.c_str());
    }
    else if(type == BT::NodeType::CONDITION){
        model->InsertNewChildElement("Condition")->SetAttribute("ID",name.c_str());
    }
    else{
        model->InsertNewChildElement("Decorator")->SetAttribute("ID",name.c_str()); 
    }

    //add the ports
    auto newNode = model->LastChildElement();
    for(auto port : ports){
        if(port.second.direction() == BT::PortDirection::INPUT){
            newNode->InsertNewChildElement("input_port");
        }
        else{
            newNode->InsertNewChildElement("output_port");
        }
        auto newPort = newNode->LastChildElement();
        //if there is a default val, add it
        if(port.second.defaultValue().size() > 0){
            newPort->SetAttribute("default", port.second.defaultValue().c_str());
        }
        //set the name
        newPort->SetAttribute("name",port.first.c_str());
        newPort->SetAttribute("required","false");
        newPort->SetText(port.second.description().c_str());
    }
}

/**
 * @brief Checks riptide_autonomy BT implemenations in factory against the groot workspace in doc. 
 * 
 * @param implementations The BT factory that contains the riptide_autonomy implementations
 * @param tree XMLDocument containing the tree to check
 * @return true if the Groot workspace matches with the riptide_autonomy implementation and vice versa
 * @return false if the workspace and the riptide_autonomy implementations do not match up.
 */
bool CheckManifest(std::shared_ptr<BehaviorTreeFactory> implementations, XMLDocument& tree, const std::string& fileName) {
    bool has_errors = false;

    //load tree node manifest
    auto root = tree.RootElement();

    //get the TreeNodesModel element to check actions/conditions
    auto model = root->LastChildElement("TreeNodesModel");
    XMLElement *node;
    std::list<std::string> nodesfound; //list of nodes in the groot workspace

    node = model->FirstChildElement();
    RCLCPP_INFO(log, "Checking node manifest of file %s", fileName.c_str());

    while(node != nullptr){
        std::string nodetype = node->Name();
        XMLElement *nextElement = node->NextSiblingElement();

        //if node is not a subtree...
        if(nodetype.compare("SubTree") != 0){
            // res = {node_error, port_error}
            std::pair<bool, bool> res = findNodeInManifest(node, implementations);
            bool removed = false;
            if(!res.first) {
                nodesfound.push_back(nodeId(node));
            }
            else{
                RCLCPP_WARN(log,"Riptide_autonomy does not have a valid c++ file for %s. Would you like to remove it from the workspace? (Y/n): ", nodeId(node));
                std::string ans;
                std::cin >> ans;

                if(std::tolower(ans[0]) == 'y' || yta){
                    //delete from the model we are iterating through
                    node->Parent()->DeleteChild(node);
                    removed = true;
                } else {
                    RCLCPP_ERROR(log, "Not removing %s from the workspace. To fix this issue, use the BT assistant tool to create a proper C++ file for the node.", nodeId(node));
                }
            }
            
            //only update the error state if the node has not been removed
            if(!removed) {
                has_errors = has_errors || res.first || res.second;
            }
        }

        node = nextElement;
    }
        
    std::list<std::string> customNodes = getCustomNodes(); //list of nodes in riptide_autonomy2
    for(std::string nodeName : customNodes){
        RCLCPP_INFO(log,"Checking for node %s in groot",nodeName.c_str());

        //check if node exists in the workspace
        if(std::find(nodesfound.begin(), nodesfound.end(), nodeName) == nodesfound.end()){
            RCLCPP_WARN(log, "%s was not found in groot workspace. Would you like to add it? (Y/n)", nodeName.c_str());
            std::string ans;
            std::cin >> ans;

            if(std::tolower(ans[0]) == 'y' || yta){
                auto node_info = implementations->manifests().at(nodeName);
                addNodeToWorkspace(tree, nodeName.c_str(), node_info.ports, node_info.type);
            }
            else{
                has_errors = true;
            }
        }
        else{
            RCLCPP_INFO(log, "\tOK!");
        }
    }

    return !has_errors;
}

/**
 * @brief Recursively traverses a tree scanning for possible blackboard issues.
 * @param doc Document containing the tree being checked. Will be modified if user approves fixes.
 * @param testTree The tree to check for issues.
 * @param treeNodesModel a pointer to the TreeNodesModel element tag
 * @param blackboard A list containing the names of known blackboard entries in the tree.
 * @return true if the check was a success. false if there are outstanding issues.
 */
bool traverseTree(XMLDocument& doc, XMLElement *testTree, XMLElement *treeNodesModel, std::list<std::string>& blackboard){
    XMLElement *tree = testTree;    
    
    if(tree == nullptr){
        return true;
    }

    bool noErrors = true;
    while(tree != nullptr){
        //add any output or inout ports to the blackboard
        if(const char *treeId = tree->Attribute("ID")) {
            //if the tree has an ID, try to find it in the node model
            XMLElement *nodeModel = treeNodesModel->FirstChildElement();
            while(nodeModel != nullptr) {
                const char *nodeModelId = nodeModel->Attribute("ID");
                if(nodeModelId) {
                    if(strcmp(nodeModelId, treeId) == 0) {
                        //found the node in the model. now find all inout and output ports and add their values to blackboard
                        XMLElement *nextPort = nodeModel->FirstChildElement();
                        while(nextPort != nullptr) {
                            const char *portType = nextPort->Value();
                            if(strcmp(portType, "output_port") == 0 || strcmp(portType, "inout_port") == 0) {
                                //found an out port. whats its name?
                                if(const char *portName = nextPort->Attribute("name")) {
                                    //now, what value was passed to it? thats the name of our blackboard entry
                                    if(const char *portVal = tree->Attribute(portName)) {
                                        //strip the {} off of the port, then add to list
                                        std::string stripped(portVal);
                                        if(stripped.back() == '}') {
                                            if(stripped[0] == '{') {
                                                stripped = stripped.substr(1, stripped.length() - 2);
                                            } else if(stripped[0] == '$' && stripped[1] == '{') {
                                                stripped = stripped.substr(2, stripped.size() - 3);
                                            }
                                        }
                                        
                                        blackboard.push_back(stripped);
                                        RCLCPP_INFO(log, "Found bb value \"%s\" from node %s", stripped.c_str(), treeId);
                                    }
                                }
                            }

                            nextPort = nextPort->NextSiblingElement();
                        }
                    }
                }

                nodeModel = nodeModel->NextSiblingElement();
            }
        } else if(strcmp(tree->Value(), "SetBlackboard") == 0) {
            //set add the name of the blackboard entry to the list
            if(const char *bbEntryName = tree->Attribute("output_key")) {
                RCLCPP_INFO(log, "Found bb value \"%s\" from node SetBlackboard", bbEntryName);
                blackboard.push_back(bbEntryName);
            }
        }

        //check that if the node is a subtree, all parameters going to it are valid blackboard pointers.
        //if there is a pointer that doesnt exist, prompt user, then create one with the value being the name
        if(std::strcmp(tree->Value(), "SubTree") == 0){
            //check to make sure ports are blackboard vals;
            //get fist attribute skipping the name
            auto attribute = tree->FirstAttribute()->Next();
            while(attribute){
                //TODO: we do want to take this into account but thats gonna kinda be an edge case ngl
                if(std::strcmp(attribute->Name(), "__shared_blackboard") != 0) { //name != __shared_blackboard
                    // RCLCPP_INFO(log, "%s", attribute->Name());
                    if(!(std::find(blackboard.begin(), blackboard.end(), attribute->Value()) != blackboard.end())){
                        RCLCPP_WARN(log, "%s is listed as a port value for %s but may not exist.", attribute->Value(), nodeId(tree));
                        RCLCPP_WARN(log,"Would you like to make one? (Y/n)");
                        std::string ans;
                        std::cin >> ans;
                        if(std::tolower(ans[0]) == 'y' || yta){
                            XMLElement *newNode = doc.NewElement("SetBlackboard");
                            newNode->SetAttribute("output_key", attribute->Value());
                            newNode->SetAttribute("value", attribute->Value());
                            tree->Parent()->InsertFirstChild(newNode);
                        } else {
                            noErrors = false;
                        }
                    }
                }
                attribute = attribute->Next();
            }
        }

        //can add more checks here as needed

        //if node has children, check them here
        noErrors = traverseTree(doc, tree->FirstChildElement(), treeNodesModel, blackboard) && noErrors;
        tree = tree->NextSiblingElement();
    }

    return noErrors;
}

/**
 * @brief Checks a specific behavior tree. Performs the standard manifest check as well as checks for issues relating to the actual tree (like blackboard)
 * @param implementations riptide_autonomy node implementations.
 * @param fileName Name of the file containing the behavior tree to check.
 * @return true if the check succeeded. false if there are outstanding issues.
 */
bool CheckTree(std::shared_ptr<BehaviorTreeFactory> implementations, XMLDocument& doc, const std::string& fileName) {
    bool noErrors = true;
    XMLElement *tree = doc.RootElement()->FirstChildElement();
    XMLElement *treeNodesModel = findFirstChildByName(doc.RootElement(), "TreeNodesModel");

    //traverse all behavior trees in the file and look for blackboard issues
    while(tree != nullptr) {
        RCLCPP_INFO(log, "Check subtree %s", nodeId(tree));
        std::list<std::string> blackboard;

        //if the node is a subtree, add its ports to the blackboard
        if(const char *name = tree->Attribute("ID")) {
            //find subtree in root/TreeNodesModel
            XMLElement *st = treeNodesModel->FirstChildElement();
            while(st != nullptr) {
                if(const char *stName = st->Attribute("ID")) {
                    if(strcmp(st->Value(), "SubTree") == 0 && strcmp(stName, name) == 0) {
                        //found it! Iterate through input ports and add to blackboard
                        XMLElement *port = st->FirstChildElement();
                        while(port != nullptr) {
                            if(strcmp(port->Value(), "input_port") == 0) {
                                if(const char *portName = port->Attribute("name")) {
                                    blackboard.push_back(portName);
                                    RCLCPP_INFO(log, "Found subtree port %s", portName);
                                }
                            }

                            port = port->NextSiblingElement();
                        }

                        break; //ensures st can never be nullptr if subtree exists and makes check after while valid
                    }
                }

                st = st->NextSiblingElement();
            }
        }

        noErrors = traverseTree(doc, tree->FirstChildElement(), treeNodesModel, blackboard) && noErrors; //noErrors LAST because short-circuit logic
        tree = tree->NextSiblingElement();
    }

    noErrors = noErrors || CheckManifest(implementations, doc, fileName);
    return noErrors;
}


int main(int argc, char **argv) {
    const std::string HOME = getEnvVar("HOME");
    if(HOME.length() == 0) {
        RCLCPP_ERROR(log, "Failed to find HOME directory!");
        return 1; //cant do anything more without HOME directory
    }

    std::string treePath = "";
    XMLDocument tree;
    bool noErrors = true;

    auto factory = std::make_shared<BT::BehaviorTreeFactory>();
    registerPluginsForFactory(factory, AUTONOMY_PKG_NAME);
    RCLCPP_INFO(log, "Loaded implementations.");

    for(int i = 1; i < argc; i++){
        if(strcmp(argv[i], "-y") == 0){
            yta = true;
        } else {
            treePath = argv[i];
            tree.LoadFile(argv[i]);
            if(!tree.RootElement()) {
                //if there is no root element then the test tree was not loaded correctly
                RCLCPP_ERROR(log, "FATAL: Could not load tree. Does it exist?");
                RCLCPP_ERROR(log, "Looked in file path: %s", treePath.c_str());
                return 1; //bail out; nothing else we can do here
            }
            RCLCPP_INFO(log, "Loaded file %s", treePath.c_str());
            noErrors = CheckTree(factory, tree, treePath) && noErrors; //&& noErrors LAST to avoid short-circuit logic
        }
    }

    //tree did not load. load the workspace and check that.
    if(!tree.RootElement()){
        treePath = HOME + std::string(AUTONOMY_WORKSPACE);
        tree.LoadFile(treePath.c_str());
        if(!tree.RootElement()) {
            //if there is no root element then the testTree was not loaded correctly
            RCLCPP_ERROR(log, "FATAL: Could not load Groot workspace file (%s). Does it exist?", treePath.c_str());
            return 1; //bail out; nothing else we can do here
        }

        RCLCPP_INFO(log, "Loaded workspace (%s)", treePath.c_str());
        noErrors = CheckManifest(factory, tree, treePath) && noErrors;
    }

    //attempt to save the file after checks are complete
    bool fileSaved = tree.SaveFile(treePath.c_str()) == XML_SUCCESS;
    if(fileSaved) {
        RCLCPP_INFO(log, "Tree file saved to %s.", treePath.c_str());
    } else {
        RCLCPP_ERROR(log, "Error saving to file %s!", treePath.c_str());
        noErrors = false;
    }

    if(!noErrors) {
        RCLCPP_WARN(log, "Some tree issues remain unresolved. Tree execution may be less reliable.");
    }

    return (noErrors ? 0 : 2);
}
