#include "riptide_autonomy/autonomy_lib.hpp"
#include "tinyxml/tinyxml2.h"

#ifndef AUTONOMY_PKG_NAME
#define AUTONOMY_PKG_NAME "riptide_autonomy2"
#endif

#define AUTONOMY_WORKSPACE "/osu-uwrt/riptide_software/src/riptide_autonomy/trees/.groot/workspace.xml"

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
 * @brief Checks riptide_autonomy BT implemenations in factory against the groot workspace in doc. 
 * 
 * @param doc The XML document containing the Groot workspace.
 * @param factory The BT factory that contains the riptide_autonomy implementations
 * @return true if the Groot workspace matches with the riptide_autonomy implementation and vice versa
 * @return false if the workspace and the riptide_autonomy implementations do not match up.
 */
bool doCheck(XMLDocument& doc, std::shared_ptr<BehaviorTreeFactory> factory) {
    auto root = doc.RootElement();


    (void) factory;
    return true;
}


int main() {
    //get HOME directory location and figure out where the groot workspace is
    const std::string HOME = getEnvVar("HOME");
    if(HOME.length() == 0) {
        std::cerr << "Failed to find HOME directory!" << std::endl;
        return 1; //cant do anything more without HOME directory
    }

    std::string workspaceLoc = HOME + std::string(AUTONOMY_WORKSPACE);

    //load Groot workspace
    XMLDocument grootWorkspace;
    grootWorkspace.LoadFile(workspaceLoc.c_str());
    if(!grootWorkspace.RootElement()) {
        //if there is no root element then the tree was not loaded correctly
        std::cerr << "FATAL: Could not load Groot workspace file (riptide_autonomy/trees/.groot/workspace.xml). Does it exist?" << std::endl;
        std::cerr << "Looked in file path: " << workspaceLoc << std::endl;
        return 2; //bail out; nothing else we can do here
    }

    //load behavior tree factory
    auto factory = std::make_shared<BT::BehaviorTreeFactory>();
    registerPluginsForFactory(factory, AUTONOMY_PKG_NAME);

    bool success = doCheck(grootWorkspace, factory);

    if(!success) {
        std::cerr << "Some workspace issues are unresolved. Tree execution may be less reliable." << std::endl;
        return 3;
    }

    return 0;
}
