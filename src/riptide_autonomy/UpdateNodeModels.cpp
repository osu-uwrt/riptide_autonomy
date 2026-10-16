/**
 * updateNodeModels: rewrites trees/lib/node_models.xml, the node list the behavior tree editor shows, from the
 * nodes this package's plugins register: every Action, Condition and Decorator with its ports and descriptions.
 * SubTree entries (the ports each subtree takes) are kept, since only the trees know them.
 *
 *   ros2 run riptide_autonomy2 updateNodeModels            rewrite the source tree's node_models.xml
 *   ros2 run riptide_autonomy2 updateNodeModels --check    exit 1 if it is out of date, change nothing
 *   ros2 run riptide_autonomy2 updateNodeModels FILE       another file
 *
 * The file is written in the editor's format (4-space indent, attributes in order), so regenerating it gives a
 * clean diff. No tree includes it; BehaviorTree.CPP never reads it.
 */
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/xml_parsing.h>
#include <tinyxml2.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <map>
#include <vector>
#include <sstream>

#include "riptide_autonomy/autonomy_lib.hpp"

#ifndef AUTONOMY_PKG_NAME
#define AUTONOMY_PKG_NAME "riptide_autonomy2"
#endif
#define AUTONOMY_TREE_DIR \
    std::string(__FILE__).substr(0, std::string(__FILE__).find("/riptide_autonomy/")) + std::string("/riptide_autonomy/trees")

namespace {

std::string escape(const std::string& text, bool attribute) {
    std::string out;
    for(const char c : text) {
        if(c == '&') out += "&amp;";
        else if(c == '<') out += "&lt;";
        else if(c == '>') out += "&gt;";
        else if(attribute && c == '"') out += "&quot;";
        else if(attribute && c == '\n') out += "&#10;";
        else out += c;
    }
    return out;
}

// The editor's canonical writer: elements, attributes in order, trimmed text; no comments needed here.
void write(std::ostringstream& out, const tinyxml2::XMLElement* e, int depth) {
    const std::string indent(static_cast<std::size_t>(depth) * 4, ' ');
    out << indent << '<' << e->Name();
    for(auto* a = e->FirstAttribute(); a; a = a->Next()) {
        out << ' ' << a->Name() << "=\"" << escape(a->Value(), true) << '"';
    }
    std::string text = e->GetText() ? e->GetText() : "";
    const auto first = text.find_first_not_of(" \t\r\n");
    text = first == std::string::npos ? "" : text.substr(first, text.find_last_not_of(" \t\r\n") - first + 1);
    if(!e->FirstChildElement()) {
        if(text.empty()) {
            out << "/>\n";
        } else {
            out << '>' << escape(text, false) << "</" << e->Name() << ">\n";
        }
        return;
    }
    out << ">\n";
    // Ports by name: BehaviorTree.CPP lists them in hash order, which would reshuffle the file on every rebuild.
    std::vector<const tinyxml2::XMLElement*> children;
    for(auto* c = e->FirstChildElement(); c; c = c->NextSiblingElement()) {
        children.push_back(c);
    }
    std::stable_sort(children.begin(), children.end(), [](auto* a, auto* b) {
        return std::string(a->Attribute("name") ? a->Attribute("name") : "") <
               std::string(b->Attribute("name") ? b->Attribute("name") : "");
    });
    for(auto* c : children) {
        write(out, c, depth + 1);
    }
    out << indent << "</" << e->Name() << ">\n";
}

std::string generate(const std::string& existingText) {
    auto factory = std::make_shared<BT::BehaviorTreeFactory>();
    registerPluginsForFactory(factory, AUTONOMY_PKG_NAME);

    tinyxml2::XMLDocument generated;
    if(generated.Parse(BT::writeTreeNodesModelXML(*factory, false).c_str()) != tinyxml2::XML_SUCCESS) {
        throw std::runtime_error("BehaviorTree.CPP wrote a model that does not parse");
    }

    // Entries by ID: the generated nodes, plus the existing file's SubTree entries.
    std::map<std::string, const tinyxml2::XMLElement*> entries;
    auto* model = generated.RootElement()->FirstChildElement("TreeNodesModel");
    for(auto* e = model ? model->FirstChildElement() : nullptr; e; e = e->NextSiblingElement()) {
        entries[e->Attribute("ID")] = e;
    }
    tinyxml2::XMLDocument existing;
    if(!existingText.empty() && existing.Parse(existingText.c_str()) == tinyxml2::XML_SUCCESS) {
        auto* old = existing.RootElement() ? existing.RootElement()->FirstChildElement("TreeNodesModel") : nullptr;
        for(auto* e = old ? old->FirstChildElement("SubTree") : nullptr; e; e = e->NextSiblingElement("SubTree")) {
            if(e->Attribute("ID") && !entries.count(e->Attribute("ID"))) {
                entries[e->Attribute("ID")] = e;
            }
        }
    }

    std::ostringstream out;
    out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<root BTCPP_format=\"4\">\n    <TreeNodesModel>\n";
    for(const auto& [id, e] : entries) {
        write(out, e, 2);
    }
    out << "    </TreeNodesModel>\n</root>\n";
    return out.str();
}

}

int main(int argc, char** argv) {
    bool check = false;
    std::filesystem::path file = std::filesystem::path(AUTONOMY_TREE_DIR) / "lib" / "node_models.xml";
    for(int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if(arg == "--check") {
            check = true;
        } else if(arg == "-h" || arg == "--help") {
            std::cout << "usage: updateNodeModels [--check] [node_models.xml]\n";
            return 0;
        } else {
            file = arg;
        }
    }

    std::string existing;
    if(std::ifstream in{file}) {
        std::stringstream buffer;
        buffer << in.rdbuf();
        existing = buffer.str();
    }

    try {
        const std::string text = generate(existing);
        if(text == existing) {
            std::cout << file.string() << " is up to date\n";
            return 0;
        }
        if(check) {
            std::cerr << file.string() << " is out of date: run updateNodeModels\n";
            return 1;
        }
        std::filesystem::create_directories(file.parent_path());
        std::ofstream(file) << text;
        std::cout << "wrote " << file.string() << '\n';
        return 0;
    } catch(const std::exception& e) {
        std::cerr << "updateNodeModels: " << e.what() << '\n';
        return 2;
    }
}
