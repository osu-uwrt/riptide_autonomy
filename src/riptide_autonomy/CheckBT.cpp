#include "riptide_autonomy/autonomy_lib.hpp"
#include "tinyxml/tinyxml2.h"

#ifndef AUTONOMY_PKG_NAME
#define AUTONOMY_PKG_NAME "ritpide_autonomy2"
#endif

using namespace tinyxml2;
using namespace BT;

bool doCheck(XMLDocument& doc, std::shared_ptr<BehaviorTreeFactory> factory) {
    (void) doc;
    (void) factory;
    return true;
}

int main() {
    //load Groot workspace
    XMLDocument doc;
    doc.LoadFile("/home/brach/osu-uwrt/riptide_software/src/riptide_autonomy/trees/.groot/workspace.xml");

    //load behavior tree factory
    auto factory = std::make_shared<BT::BehaviorTreeFactory>();
    registerPluginsForFactory(factory, AUTONOMY_PKG_NAME);

    bool success = doCheck(doc, factory);

    if(!success) {
        std::cerr << "Some workspace issues are unresolved. Tree execution may be less reliable." << std::endl;
        return 1;
    }

    return 0;
}
