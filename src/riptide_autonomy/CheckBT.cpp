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

std::string treePath = "";
XMLDocument tree;
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
bool addPortToWorkspace(std::string missingPort, PortInfo info, XMLElement *node){


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


    return (tree.SaveFile(treePath.c_str()) == XML_SUCCESS);
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
                RCLCPP_ERROR(log, "%s was not found in groot workspace for node %s. Would you like to add it? (Y/n)",check.first.c_str(),node->Attribute("ID"));
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
    bool port_error= true;
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
        // node exists! check to make sure ports match
        port_error = checkPorts(test, node_maifest.ports);
    }
    catch(...){        
    }
    return {node_error, port_error};
}




/**
 * @brief Creates and adds XML element contining node information to the groot workspae
 * 
 * @param name the name of the node to add
 * @param ports the assocaited ports list contianing all ports and all relevent information for each port
 * @param type the node type (Action, Condition, Decorator)
 */
bool addNodeToWorkspace(std::string name, PortsList ports, BT::NodeType type){
    //load Groot workspace
    auto model = tree.RootElement()->LastChildElement();
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

    return (tree.SaveFile(treePath.c_str()) == XML_SUCCESS);

}
/**
 * @brief Checks riptide_autonomy BT implemenations in factory against the groot workspace in doc. 
 * 
 * @param doc The XML document containing the Groot workspace.
 * @param factory The BT factory that contains the riptide_autonomy implementations
 * @return true if the Groot workspace matches with the riptide_autonomy implementation and vice versa
 * @return false if the workspace and the riptide_autonomy implementations do not match up.
 */
void CheckWorkspace(std::shared_ptr<BehaviorTreeFactory> factory) {
    bool has_errors = false;
    //load Groot workspace
    auto root = tree.RootElement();
    //get the TreeNodesModel element to check actions/conditions
    auto model = root->LastChildElement();
    XMLElement *node;
    std::list<std::string> nodesfound;

    node = model->FirstChildElement();

    while(node != nullptr){
        std::string nodetype = node->Name();
        if(nodetype.compare("SubTree")!=0){
            std::pair<bool,bool> res = findNodeInManifest(node,factory);
            has_errors = !res.first && !res.second;
            if(!res.first){
                nodesfound.push_back(node->Attribute("ID"));
            }   
            else{
                RCLCPP_WARN(log,"Riptide_autonomy does not have a valid c++ file for %s. Would you like to remove it from the workspace? (Y/n): ", node->Attribute("ID"));
                std::string ans;
                std::cin>>ans;

                if(std::tolower(ans[0]) == 'y'||yta){
                    has_errors = false || res.second;
                    node = node->PreviousSiblingElement();
                    node->Parent()->DeleteChild(node->NextSiblingElement());
                }
            }        
        }

        node = node->NextSiblingElement();       
    }
        
    std::list<std::string> customNodes = getCustomNodes();
    for(std::string node : customNodes){
        RCLCPP_INFO(log,"Checking for node %s in groot",node.c_str());
        if(!(std::find(nodesfound.begin(), nodesfound.end(), node) != nodesfound.end())){
            RCLCPP_ERROR(log, "%s was not found in groot workspace. Would you like to add it? (Y/n)",node.c_str());
            std::string ans;
            std::cin>>ans;

            if(std::tolower(ans[0]) == 'y' || yta){
                auto node_info = factory->manifests().at(node);
                addNodeToWorkspace(node.c_str(), node_info.ports,node_info.type);
            }
            else{
                has_errors = true;
            }
        }
        else{
            RCLCPP_INFO(log, "\tOK!");
        }
    }


    if(has_errors) {
        RCLCPP_WARN(log, "Some workspace issues are unresolved. Tree execution may be less reliable.");
    }
}

bool traverseTree(XMLElement *testTree, XMLElement *bb, std::list<std::string> blackboard){
    if(testTree == nullptr){
        return true;
    }
    while(testTree != nullptr){
        if(std::strcmp(testTree->Value(), "SetBlackboard") == 0){
            //add blackboard vals to the list
            if(bb->Parent() != testTree->Parent()){
                blackboard.clear();
            }
            blackboard.push_back(testTree->Attribute("output_key"));
            bb = testTree;
        }
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
                        std::cin>>ans;
                        if(std::tolower(ans[0]) == 'y' || yta){
                            XMLElement *newNode = tree.NewElement("SetBlackboard");
                            newNode->SetAttribute("output_key", attribute->Value());
                            newNode->SetAttribute("value", attribute->Value());
                            bb->Parent()->InsertAfterChild(bb,newNode);
                            tree.SaveFile(treePath.c_str());
                        }
                    }
                }
                attribute = attribute->Next();
            }
        }
        //can add more checks here as needed

        traverseTree(testTree->FirstChildElement(), bb, blackboard);
        testTree = testTree->NextSiblingElement();
    }
    return true;
}

bool CheckTree(){
    auto mainTree = tree.RootElement()->FirstChildElement();
    //find main testTree. ID="BehaviorTree"
    while(mainTree !=nullptr &&  std::strcmp(mainTree->Attribute("ID"),"BehaviorTree")){
        mainTree = mainTree->NextSiblingElement();
    }
    std::list<std::string> blackboard;
    traverseTree(mainTree->FirstChildElement(), mainTree->FirstChildElement(), blackboard);
    return true;
}


int main(int argc, char **argv) {

    const std::string HOME = getEnvVar("HOME");
    if(HOME.length() == 0) {
        std::cerr << "Failed to find HOME directory!" << std::endl;
        return 1; //cant do anything more without HOME directory
    }
    for(int i = 1; i < argc; i++){
        if(argv[i]  == "-y"){
            yta=true;
        }
        else{
        treePath = argv[i];
        tree.LoadFile(argv[i]);
        if(!tree.RootElement()) {
            //if there is no root element then the testTree was not loaded correctly
            std::cerr << "FATAL: Could not load testTree. Does it exist?" << std::endl;
            std::cerr << "Looked in file path: " << treePath << std::endl;
            return 1; //bail out; nothing else we can do here
        }
        RCLCPP_INFO(log, "Workspace loaded");
        CheckTree();
        }
    }
    if(!tree.RootElement()){
        treePath = HOME + std::string(AUTONOMY_WORKSPACE);
        tree.LoadFile(treePath.c_str());
        if(!tree.RootElement()) {
            //if there is no root element then the testTree was not loaded correctly
            std::cerr << "FATAL: Could not load Groot workspace file (riptide_autonomy/trees/.groot/workspace.xml). Does it exist?" << std::endl;
            std::cerr << "Looked in file path: " << treePath << std::endl;
            return 1; //bail out; nothing else we can do here
        }
        RCLCPP_INFO(log, "Workspace loaded");
        //load behavior testTree factory
    }
    auto factory = std::make_shared<BT::BehaviorTreeFactory>();
    registerPluginsForFactory(factory, AUTONOMY_PKG_NAME);
    CheckWorkspace(factory);   
    return 0;
}