#include <filesystem>

#include "riptide_autonomy/autonomy_lib.hpp"
#include "riptide_autonomy/tinyxml2.h"

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
 * @brief Prompts the user with a message and a list of options to choose from, then collects and returns the users input.
 * If any user input is invalid, the user will be re-prompted.
 * 
 * @param query The message to prompt the user with.
 * @param options The options to give the user
 * @return int the index in the vector of the option that the user chose.
 */
int prompt(const std::string query, const std::vector<std::string> options) {
    //print query and options
    std::cout << query << std::endl;
    for(uint i = 0; i < options.size(); i++) {
        //use i + 1 so that options start at 1
        std::cout << "  " << i + 1 << ": " << options[i] << std::endl;
    }

    int selectedOption = 0;
    while(selectedOption <= 0 || abs(selectedOption) > options.size()) {
        std::string userInput;
        std::cout << "[1 - " << options.size() << "]? ";
        std::cin >> userInput;

        //attempt to convert user input to a valid integer and check the range
        try {
            selectedOption = std::stoi(userInput);            
        } catch(std::invalid_argument& ex) {
            std::cout << "Please enter a valid number." << std::endl;
            continue;
        }

        if(selectedOption <= 0 || abs(selectedOption) > options.size()) {
            std::cout << "Please enter a number between 1 and " << options.size() << "." << std::endl;
        }
    }
    
    return selectedOption - 1;
}

/**
 * @brief Checks to see if a certain XML element is found in riptide_autonomy 
 * 
 * @param port The XML node in the groot workspace
 * @param ports TThe list of ports the riptide_autonomy implementation has
 * @return true if the XML node is in the PortsList
 * @return false if it is not int the list, or if the directions do not match.
 */
bool findPortInManifest(XMLElement *port, PortsList ports){
    try{
        
        std::string type = port->Name();
        if(type.compare("inout_port") == 0){
            return true;
        }
        else if(type.compare("output_port") == 0){
            if(ports.at(port->Attribute("name")).direction() == BT::PortDirection::OUTPUT){
                return true;
            }
            RCLCPP_WARN(log, "%s is listed as an output port in groot but was not implemented as such.", port->Attribute("name"));
        }
        else{
            if(ports.at(port->Attribute("name")).direction() == BT::PortDirection::INPUT){
                return true;
            }
            RCLCPP_WARN(log, "%s is listed as an input port in groot but was not implemented as such.", port->Attribute("name"));
        }                    
    }
    catch(const std::out_of_range &e){
        RCLCPP_ERROR(log,"%s in workspace is not in ripride_autonomy", port->Attribute("name"));
    }
    return false;
}

/**
 * @brief Goes through Include to get all custom nodes
 * 
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
bool findNodeInManifest(XMLElement *test, std::shared_ptr<BehaviorTreeFactory> factory){
    bool exists = false;
    bool has_errors = false;
    //arbitrarily return true for subtrees we only care about conditions/actions/decorators
    std::string testNodeType = test->Name();
    
    if(testNodeType.compare("SubTree") !=0){
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
                auto port = test->FirstChildElement();
                while(port != test->LastChildElement()){
                    //try to find the port in the manifest. if it errors its not there, otherwise make sure their directions match
                    if(findPortInManifest(port,ports)){
                        portsfound.push_back(port->Attribute("name"));
                    }
                    else{
                        has_errors = true;
                    }
                    //go to next port
                    port = port->NextSiblingElement();
                }
                //do it one extra time to get the last port
                if(findPortInManifest(port,ports)){
                    portsfound.push_back(port->Attribute("name"));
                }
                // check the other way. make sure every port in the manifest is in groot
                if(ports.size() != portsfound.size()){
                    for(auto check : ports){
                        if(!(std::find(portsfound.begin(), portsfound.end(), check.first) != portsfound.end())){
                            RCLCPP_ERROR(log, "%s was not found in groot workspace for node %s",check.first.c_str(),test->Attribute("ID"));
                            has_errors = true;
                        }
                    }
                }
            }
            
        }
        if(!exists){
            RCLCPP_ERROR(log, "%s does not have a valid implementation",test->Attribute("ID"));
        }
        if(!has_errors && exists){
            RCLCPP_INFO(log, "\tOK!");
        }
        return !has_errors && exists;
    }
    return true;
}


/**
 * @brief Checks riptide_autonomy BT implemenations in factory against the groot workspace in doc. 
 * 
 * @param doc The XML document containing the Groot workspace.
 * @param factory The BT factory that contains the riptide_autonomy implementations
 * @return true if the Groot workspace matches with the riptide_autonomy implementation and vice versa
 * @return false if the workspace and the riptide_autonomy implementations do not match up.
 */
bool doCheck(std::string doc, std::shared_ptr<BehaviorTreeFactory> factory) {
    bool has_errors = false;
    //load Groot workspace
    XMLDocument grootWorkspace;
    grootWorkspace.LoadFile(doc.c_str());
    if(!grootWorkspace.RootElement()) {
        //if there is no root element then the tree was not loaded correctly
        std::cerr << "FATAL: Could not load Groot workspace file (riptide_autonomy/trees/.groot/workspace.xml). Does it exist?" << std::endl;
        std::cerr << "Looked in file path: " << doc << std::endl;
        return 2; //bail out; nothing else we can do here
    }
    RCLCPP_INFO(log, "Workspace loaded");
    auto root = grootWorkspace.RootElement();
    //get the TreeNodesModel element to check actions/conditions
    auto model = root->LastChildElement();

    auto node = model->FirstChildElement();

    std::list<std::string> nodesfound = getCustomNodes();
    while(node != model->LastChildElement()){
        if(findNodeInManifest(node,factory)){
            nodesfound.push_back(node->Attribute("ID"));
        }
        else{
            has_errors =true;           
        }
        node = node->NextSiblingElement();       
    }

    std::list<std::string> customNodes = getCustomNodes();
    for(std::string node : customNodes){
        RCLCPP_INFO(log,"Checking for node %s in groot",node.c_str());
        if(!(std::find(nodesfound.begin(), nodesfound.end(), node) != nodesfound.end())){
            RCLCPP_ERROR(log, "%s was not found in groot workspace",node.c_str());
            has_errors = true;
        }
        else{
            RCLCPP_INFO(log, "\tOK!");
        }
    }

    return !has_errors;
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

    bool success = doCheck(workspaceLoc, factory);

    if(!success) {
        std::cerr << "Some workspace issues are unresolved. Tree execution may be less reliable." << std::endl;
        return 3;
    }

    return 0;
}
