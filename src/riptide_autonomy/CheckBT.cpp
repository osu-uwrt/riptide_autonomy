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
    std::string type = port->Name();
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
    std::list<std::string> ports_found;
    while(port != nullptr){
        try{
            PortInfo info = src_ports.at(port->Attribute("name"));
            ports_found.push_back(port->Attribute("name"));
            if(!checkPort(info, port)){
                port_match = false;
            }             

        }
        catch(...){
            RCLCPP_WARN(log, "Port %s in groot was not found in c++ implementation. Would you like to remove it? (Y/n): ", port->Attribute("name"));
            std::string ans;
            std::cin>>ans;

            if(std::tolower(ans[0]) == 'y' || yta){
                port->Parent()->DeleteChild(port);
            }
            else{
                port_match = false;
            }
        }
        port = port->NextSiblingElement();
    }

    // check the other way. make sure every port in the manifest is in groot
    if(src_ports.size() != ports_found.size()){
        for(auto check : src_ports){
            if(!(std::find(ports_found.begin(), ports_found.end(), check.first) != ports_found.end())){
                RCLCPP_WARN(log, "%s was not found in groot workspace for node %s. Would you like to add it? (Y/n)",check.first.c_str(),node->Attribute("ID"));
                std::string ans;
                std::cin>>ans;

                if(std::tolower(ans[0]) == 'y' || yta){
                    addPortToWorkspace(check.first, check.second, node);
                }
                else{
                    port_match = false;
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
    
    RCLCPP_INFO(log,"Checking Node %s", test->Attribute("ID"));
    //get the manifest
    std::unordered_map<std::string, BT::TreeNodeManifest> src_manifest = factory->manifests();    
    // try to find node in manifest
    try{
        BT::TreeNodeManifest node_maifest = src_manifest.at(test->Attribute("ID"));
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

        //if node is not a subtree...
        if(nodetype.compare("SubTree") != 0){
            // res = {node_error, port_error}
            std::pair<bool, bool> res = findNodeInManifest(node, implementations);
            bool removed = false;
            if(!res.first) {
                nodesfound.push_back(node->Attribute("ID"));
            }
            else{
                RCLCPP_WARN(log,"Riptide_autonomy does not have a valid c++ file for %s. Would you like to remove it from the workspace? (Y/n): ", node->Attribute("ID"));
                std::string ans;
                std::cin >> ans;

                if(std::tolower(ans[0]) == 'y' || yta){
                    //delete from the model we are iterating through
                    node = node->PreviousSiblingElement();
                    node->Parent()->DeleteChild(node->NextSiblingElement());
                    removed = true;
                } else {
                    RCLCPP_ERROR(log, "Not removing %s from the workspace. To fix this issue, use the BT assistant tool to create a proper C++ file for the node.", node->Attribute("ID"));
                }
            }
            
            //only update the error state if the node has not been removed
            if(!removed) {
                has_errors = has_errors || res.first || res.second;
            }
        }

        node = node->NextSiblingElement();       
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
 * @param bb The location in the tree to insert SetBlackboard nodes as needed.
 * @param blackboard A list containing the names of known blackboard entries in the tree.
 * @return true if the check was a success. false if there are outstanding issues.
 */
bool traverseTree(XMLDocument& doc, XMLElement *testTree, XMLElement *bb, std::list<std::string> blackboard){
    if(testTree == nullptr){
        return true;
    }

    bool noErrors = true;
    while(testTree != nullptr){
        //if the node sets a blackboard pointer, record it
        if(std::strcmp(testTree->Value(), "SetBlackboard") == 0){
            //add blackboard vals to the list
            if(bb->Parent() != testTree->Parent()){
                blackboard.clear();
            }
            blackboard.push_back(testTree->Attribute("output_key"));
            bb = testTree;
        }

        //check that if the node is a subtree, all parameters going to it are valid blackboard pointers.
        //if there is a pointer that doesnt exist, prompt user, then create one with the value being the name
        if(std::strcmp(testTree->Value(), "SubTree") == 0){
            RCLCPP_INFO(log, "found subtree %s", testTree->Attribute("ID"));
            //check to make sure ports are blackboard vals;
            //get fist attribute skipping the name
            auto attribute = testTree->FirstAttribute()->Next();
            while(attribute){
                if(std::strcmp(attribute->Name(), "__shared_blackboard")){
                    RCLCPP_INFO(log, "%s", attribute->Name());
                    if(!(std::find(blackboard.begin(), blackboard.end(), attribute->Value()) != blackboard.end())){
                        RCLCPP_WARN(log, "%s is listed as a port value for %s but is not a blackboard pointer.",attribute->Value(),testTree->Attribute("ID"));
                        RCLCPP_WARN(log,"Would you like to make one? (Y/n)");
                        std::string ans;
                        std::cin >> ans;
                        if(std::tolower(ans[0]) == 'y' || yta){
                            XMLElement *newNode = doc.NewElement("SetBlackboard");
                            newNode->SetAttribute("output_key", attribute->Value());
                            newNode->SetAttribute("value", attribute->Value());
                            bb->Parent()->InsertAfterChild(bb,newNode);
                        } else {
                            noErrors = false;
                        }
                    }
                }
                attribute = attribute->Next();
            }
        }

        //can add more checks here as needed

        noErrors = noErrors && traverseTree(doc, testTree->FirstChildElement(), bb, blackboard);
        testTree = testTree->NextSiblingElement();
    }

    return noErrors;
}

/**
 * @brief Checks a specific behavior tree. Performs the standard manifest check as well as checks for issues relating to the actual tree (like blackboard)
 * @param implementations riptide_autonomy node implementations.
 * @param fileName Name of the file containing the behavior tree to check.
 * @return true if the check succeeded. false if there are outstanding issues.
 */
bool CheckTree(std::shared_ptr<BehaviorTreeFactory> implementations, XMLDocument& tree, const std::string& fileName) {
    auto mainTree = tree.RootElement()->FirstChildElement();
    //find main testTree. ID="BehaviorTree"
    while(mainTree !=nullptr && std::strcmp(mainTree->Attribute("ID"),"BehaviorTree")){
        mainTree = mainTree->NextSiblingElement();
    }
    std::list<std::string> blackboard;

    bool noErrors = traverseTree(tree, mainTree->FirstChildElement(), mainTree->FirstChildElement(), blackboard);
    noErrors = noErrors || CheckManifest(implementations, tree, fileName);
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
            yta=true;
        } else {
            treePath = argv[i];
            tree.LoadFile(argv[i]);
            if(!tree.RootElement()) {
                //if there is no root element then the testTree was not loaded correctly
                RCLCPP_ERROR(log, "FATAL: Could not load testTree. Does it exist?");
                RCLCPP_ERROR(log, "Looked in file path: %s", treePath.c_str());
                return 1; //bail out; nothing else we can do here
            }
            RCLCPP_INFO(log, "Loaded file %s", treePath.c_str());
            noErrors = noErrors && CheckTree(factory, tree, treePath);
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
        noErrors = noErrors && CheckManifest(factory, tree, treePath);
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
