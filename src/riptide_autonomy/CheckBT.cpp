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

/**
 * @brief Creates and adds XML element contining node information to the groot workspae
 * 
 * @param name the name of the node to add
 * @param ports the assocaited ports list contianing all ports and all relevent information for each port
 * @param type the node type (Action, Condition, Decorator)
 * @param doc filepath to groot workspace 
 */
bool addNodeToWorkspace(std::string name, PortsList ports, BT::NodeType type,  std::string doc){
    //load Groot workspace
    XMLDocument grootWorkspace;
    grootWorkspace.LoadFile(doc.c_str());
    auto root = grootWorkspace.RootElement();
    auto model = root->LastChildElement();
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
    for(auto port :ports){
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

    return (grootWorkspace.SaveFile(doc.c_str()) == XML_SUCCESS);

}
/**
 * @brief Creates and adds XML element contining port information to the groot workspae
 * 
 * @param missingPort the name of the port to add
 * @param info  all relevent information for each port
 * @param node the nmane of the node the port belongs to
 * @param doc filepath to groot workspace 
 */
bool addPortToWorkspace(std::string missingPort, PortInfo info, std::string node,std::string doc){

    XMLDocument grootWorkspace;
    grootWorkspace.LoadFile(doc.c_str());
    auto root = grootWorkspace.RootElement();
    auto model = root->LastChildElement();
    auto nodeToAddTo = model->FirstChildElement();
    while(nodeToAddTo != nullptr){
        std::string name = nodeToAddTo->Attribute("ID");
        if(name.compare(node) == 0){
            break;
        }
        nodeToAddTo = nodeToAddTo->NextSiblingElement();
    }
    if(info.direction() == BT::PortDirection::INPUT){
    nodeToAddTo->InsertNewChildElement("input_port");
    }
    else{
        nodeToAddTo->InsertNewChildElement("output_port");
    }
    auto newPort = nodeToAddTo->LastChildElement();
    //if there is a default val, add it
    if(info.defaultValue().size() > 0){
        newPort->SetAttribute("default", info.defaultValue().c_str());
    }
    //set the name
    newPort->SetAttribute("name",missingPort.c_str());
    newPort->SetAttribute("required","false");
    newPort->SetText(info.description().c_str());   


    return (grootWorkspace.SaveFile(doc.c_str()) == XML_SUCCESS);
}

/**
 * @brief Prompts the user with a message, then collects and returns the users input.
 * If any user input is invalid, the user will be re-prompted.
 * 
 * @param nodeToAdd The name of the node to add to workspace
 * @param factory The BT factory that contains the riptide_autonomy implementations
 * @param doc The filepath to the workspace
 * @return true if the node was added
 * @return false if the node was not added 
 */
bool promptAddNode(std::string nodeToAdd, std::shared_ptr<BehaviorTreeFactory> factory,std::string doc) {
    std::unordered_map<std::string, BT::TreeNodeManifest> manifest = factory->manifests();
    std::string ans;
    char check;
    do{
    RCLCPP_INFO(log, "Would you like to add %s to the groot workspace? (y/n)",nodeToAdd.c_str());
    std::cin>>ans;
        if(ans.size() == 1){
            check = std::tolower(ans[0]);
            
        }
        if(check =='y'){
            if(addNodeToWorkspace(nodeToAdd, manifest.at(nodeToAdd).ports, manifest.at(nodeToAdd).type, doc)){
                RCLCPP_INFO(log,"Successfully added node!");
                return true;
            }
            else{
                RCLCPP_INFO(log,"There was an unexpected error adding the node :(");
                return false;
            }
        }
    }while(check != 'n');
    return false;
    
}

/**
 * @brief Prompts the user with a message, then collects and returns the users input.
 * If any user input is invalid, the user will be re-prompted.
 * 
 * @param missingPort The name of the port to add to workspace
 * @param info Information about the missing port
 * @param doc The filepath to the workspace
 * @param node The name of the node the port is associated with
 * @return true if the port was added
 * @return false if the port was not added 
 */
bool promptAddPort(std::string missingPort, PortInfo info, std::string node, std::string doc) {
    std::string ans;
    char check;
    do{
    RCLCPP_INFO(log, "Would you like to add %s to the node %s? (y/n)",missingPort.c_str(), node.c_str());
    std::cin>>ans;
    if(ans.size() ==1){
        check = std::tolower(ans[0]);
    }

    if(check =='y'){
        if(addPortToWorkspace(missingPort, info, node, doc)){
            RCLCPP_INFO(log,"Successfully added port!");
            return true;
        }
        else{
            RCLCPP_INFO(log,"There was an unexpected error adding the port :(");
            return false;
        }
    }
    } while(check!='n');
    return false;
    
}

/**
 * @brief Checks to see if a certain XML element is found in riptide_autonomy 
 * 
 * @param port The XML node in the groot workspace
 * @param nodeInfo TNode information containing all of its ports as well as the node name
 * @return true if the XML node is in the PortsList
 * @return false if it is not int the list, or if the directions do not match.
 */
bool findPortInManifest(XMLElement *port, std::pair<const std::string, BT::TreeNodeManifest> nodeInfo,std::string doc){
    try{
        
        std::string type = port->Name();
        if(type.compare("inout_port") == 0){
            nodeInfo.second.ports.at(port->Attribute("name"));
            return true;
        }
        else if(type.compare("output_port") == 0){
            if(nodeInfo.second.ports.at(port->Attribute("name")).direction() == BT::PortDirection::OUTPUT){
                return true;
            }
            RCLCPP_WARN(log, "%s is listed as an output port in groot but was not implemented as such.", port->Attribute("name"));
        }
        else{
            if(nodeInfo.second.ports.at(port->Attribute("name")).direction() == BT::PortDirection::INPUT){
                return true;
            }
            RCLCPP_WARN(log, "%s is listed as an input port in groot but was not implemented as such.", port->Attribute("name"));
        }                    
    }
    catch(const std::out_of_range &e){
        RCLCPP_ERROR(log,"%s in workspace is not in ripride_autonomy Would you like to remove it? (y/n)", port->Attribute("name"));
        std::string ans;
        std::cin>>ans;
        if(std::tolower(ans[0]) == 'y'){
            //load Groot workspace
            XMLDocument grootWorkspace;
            grootWorkspace.LoadFile(doc.c_str());
            auto nodeToDeleteFrom = grootWorkspace.RootElement()->LastChildElement()->FirstChildElement();
            while(std::strcmp(nodeInfo.first.c_str(), nodeToDeleteFrom->Attribute("ID")) != 0){
                nodeToDeleteFrom = nodeToDeleteFrom->NextSiblingElement();
            }
            auto portToDelete = nodeToDeleteFrom->FirstChildElement();
            while(std::strcmp(port->Attribute("name"), portToDelete->Attribute("name")) != 0){
                portToDelete = portToDelete->NextSiblingElement();
            }
            portToDelete->Parent()->DeleteChild(portToDelete);
            return (grootWorkspace.SaveFile(doc.c_str()) == XML_SUCCESS);
            
        }

    }
    return false;
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
 * @brief Checks to see if a certain XML element is found in riptide_autonomy 
 * 
 * @param test The XML node in the groot workspace
 * @param factory The BT factory that contains the riptide_autonomy implementations
 * @return true if the XML node exactly matches the riptide_autonomy implementation
 * @return false if there are discrepencies between the groot workspace and riptide_autonomy
 */
bool findNodeInManifest(XMLElement *test, std::shared_ptr<BehaviorTreeFactory> factory, std::string doc){
    bool exists = false;
    bool has_errors = false;
    //arbitrarily return true for subtrees we only care about conditions/actions/decorators
    std::string testNodeType = test->Name();
    
    RCLCPP_INFO(log,"Checking Node %s", test->Attribute("ID"));
    //get the manifest
    std::unordered_map<std::string, BT::TreeNodeManifest> src_manifest = factory->manifests();    
    //iterate though each node to see if one matches
    for(auto pair : src_manifest){
        //if a node is found search and check to make sure ports are the same
        if(test->Attribute("ID") == pair.first){
            exists = true;
            PortsList ports = pair.second.ports;
            //create a list to keep track of the ports we found. We will use this later to make checking the other way easier.
            std::list<std::string> portsfound;
            XMLElement *port;
            if(test->FirstChild() != NULL){
                port = test->FirstChildElement();

                while(port && port != nullptr){
                    //try to find the port in the manifest. if it errors its not there, otherwise make sure their directions match
                    if(findPortInManifest(port,pair,doc)){
                        portsfound.push_back(port->Attribute("name"));
                    }
                    else{
                        has_errors = true;
                    }
                    //go to next port
                    port = port->NextSiblingElement();
                }

            }
            // check the other way. make sure every port in the manifest is in groot
            if(ports.size() != portsfound.size()){
                for(auto check : ports){
                    if(!(std::find(portsfound.begin(), portsfound.end(), check.first) != portsfound.end())){
                        RCLCPP_ERROR(log, "%s was not found in groot workspace for node %s",check.first.c_str(),test->Attribute("ID"));
                        has_errors = promptAddPort(check.first, check.second, pair.first, doc);
                        
                    }
                }
            }
        }
    }
    if(!exists){
        RCLCPP_ERROR(log, "%s does not have a valid implementation. Would you like to remove it from the workspace? (y/n)",test->Attribute("ID"));
        std::string ans;
        std::cin>>ans;

        if(std::tolower(ans[0]) == 'y'){
            //load Groot workspace
            XMLDocument grootWorkspace;
            grootWorkspace.LoadFile(doc.c_str());
            auto nodeToDelete = grootWorkspace.RootElement()->LastChildElement()->FirstChildElement();
            while(std::strcmp(test->Attribute("ID"), nodeToDelete->Attribute("ID")) != 0){
                nodeToDelete = nodeToDelete->NextSiblingElement();
            }
            nodeToDelete->Parent()->DeleteChild(nodeToDelete);
            return (grootWorkspace.SaveFile(doc.c_str()) == XML_SUCCESS);
        }

    }
    if(!has_errors && exists){
        RCLCPP_INFO(log, "\tOK!");
    }
    return !has_errors && exists;
}




/**
 * @brief Checks riptide_autonomy BT implemenations in factory against the groot workspace in doc. 
 * 
 * @param doc The XML document containing the Groot workspace.
 * @param factory The BT factory that contains the riptide_autonomy implementations
 * @return true if the Groot workspace matches with the riptide_autonomy implementation and vice versa
 * @return false if the workspace and the riptide_autonomy implementations do not match up.
 */
void doCheck(std::string doc, std::shared_ptr<BehaviorTreeFactory> factory) {
    bool has_errors = false;
    //load Groot workspace
    XMLDocument grootWorkspace;
    grootWorkspace.LoadFile(doc.c_str());
    if(!grootWorkspace.RootElement()) {
        //if there is no root element then the tree was not loaded correctly
        std::cerr << "FATAL: Could not load Groot workspace file (riptide_autonomy/trees/.groot/workspace.xml). Does it exist?" << std::endl;
        std::cerr << "Looked in file path: " << doc << std::endl;
        return; //bail out; nothing else we can do here
    }
    RCLCPP_INFO(log, "Workspace loaded");
    auto root = grootWorkspace.RootElement();
    //get the TreeNodesModel element to check actions/conditions
    auto model = root->LastChildElement();
    XMLElement *node;
    std::list<std::string> nodesfound;
    if(model->FirstChild() != NULL){
        node = model->FirstChildElement();

        while(node != nullptr){
            std::string nodetype = node->Name();
            if(nodetype.compare("SubTree")!=0 && !findNodeInManifest(node,factory,doc)){
                has_errors =true;           
            }
            try{
                factory->manifests().at(node->Attribute("ID"));
                nodesfound.push_back(node->Attribute("ID"));
            }
            catch(...){

            }

            node = node->NextSiblingElement();       
        }
    }      
    std::list<std::string> customNodes = getCustomNodes();
    for(std::string node : customNodes){
        RCLCPP_INFO(log,"Checking for node %s in groot",node.c_str());
        if(!(std::find(nodesfound.begin(), nodesfound.end(), node) != nodesfound.end())){
            RCLCPP_ERROR(log, "%s was not found in groot workspace",node.c_str());
            has_errors = promptAddNode(node,factory,doc);
        }
        else{
            RCLCPP_INFO(log, "\tOK!");
        }
    }


    if(has_errors) {
        RCLCPP_WARN(log, "Some workspace issues are unresolved. Tree execution may be less reliable.");
    }
}


int main() {
    //get HOME directory location and figure out where the groot workspace is
    const std::string HOME = getEnvVar("HOME");
    if(HOME.length() == 0) {
        std::cerr << "Failed to find HOME directory!" << std::endl;
        return 1; //cant do anything more without HOME directory
    }

    std::string workspaceLoc = HOME + std::string(AUTONOMY_WORKSPACE);

    //load behavior tree factory
    auto factory = std::make_shared<BT::BehaviorTreeFactory>();
    registerPluginsForFactory(factory, AUTONOMY_PKG_NAME);

    doCheck(workspaceLoc, factory);
    

    return 0;
}

