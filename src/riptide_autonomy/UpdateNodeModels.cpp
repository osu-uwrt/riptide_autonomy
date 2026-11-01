/**
 * updateNodeModels: rewrites trees/lib/node_models.xml, the node list the behavior tree editor shows, from the
 * nodes this package's plugins register: every Action, Condition and Decorator with its ports and descriptions.
 * SubTree entries (the ports each subtree takes) are kept, since only the trees know them.
 *
 *   ros2 run riptide_autonomy2 updateNodeModels            rewrite the source tree's node_models.xml
 *   ros2 run riptide_autonomy2 updateNodeModels --check    exit 1 if it is out of date, change nothing
 *   ros2 run riptide_autonomy2 updateNodeModels FILE       another file
 *   ros2 run riptide_autonomy2 updateNodeModels --builtins FILE
 *                                                          BehaviorTree.CPP's own node models (notgroot ships them)
 *
 * The file is written in the editor's format (notgroot docs/contract.md section 4), so regenerating it gives a
 * clean diff. Comments are kept, and move with the entry or port they come before. No tree includes it;
 * BehaviorTree.CPP never reads it.
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

// Comments and the element after them: sorting moves them together, so a comment stays with what it describes.
struct Item {
    std::vector<const tinyxml2::XMLComment*> comments;
    const tinyxml2::XMLElement* element;
    const tinyxml2::XMLElement* commentsFrom; // generated entry: the old entry whose inner comments it keeps
};

std::string sortKey(const tinyxml2::XMLElement* e) {
    const char* key = e->Attribute("ID") ? e->Attribute("ID") : e->Attribute("name");
    return key ? key : "";
}

void sortItems(std::vector<Item>& list) {
    // BehaviorTree.CPP lists ports in hash order, which would reshuffle the file on every rebuild
    std::stable_sort(list.begin(), list.end(), [](const Item& a, const Item& b) {
        return sortKey(a.element) < sortKey(b.element);
    });
}

// Children in order. Comments after the last element are trailing.
std::vector<Item> items(const tinyxml2::XMLNode* parent, std::vector<const tinyxml2::XMLComment*>& trailing) {
    std::vector<Item> result;
    std::vector<const tinyxml2::XMLComment*> comments;
    for(auto* n = parent ? parent->FirstChild() : nullptr; n; n = n->NextSibling()) {
        if(auto* c = n->ToComment()) {
            comments.push_back(c);
        } else if(auto* e = n->ToElement()) {
            result.push_back({comments, e, nullptr});
            comments.clear();
        }
    }
    trailing = comments;
    return result;
}

void writeComment(std::ostringstream& out, const tinyxml2::XMLComment* c, int depth) {
    out << std::string(static_cast<std::size_t>(depth) * 4, ' ') << "<!--" << c->Value() << "-->\n";
}

// TreeNodesModel's merged entries, written in place of the file's own
struct Model {
    const tinyxml2::XMLElement* replaces;
    std::vector<Item> entries;
    std::vector<const tinyxml2::XMLComment*> trailing;
};

void write(std::ostringstream& out, const tinyxml2::XMLElement* e, int depth, const Model& model, const tinyxml2::XMLElement* commentsFrom = nullptr);

void writeItems(std::ostringstream& out, std::vector<Item> list, const std::vector<const tinyxml2::XMLComment*>& trailing, int depth, const Model& model) {
    for(const auto& item : list) {
        for(auto* c : item.comments) {
            writeComment(out, c, depth);
        }
        write(out, item.element, depth, model, item.commentsFrom);
    }
    for(auto* c : trailing) {
        writeComment(out, c, depth);
    }
}

// The editor's canonical writer (notgroot docs/contract.md section 4): elements, attributes in order, trimmed
// text, comments verbatim. TreeNodesModel's entries and their ports are sorted.
void write(std::ostringstream& out, const tinyxml2::XMLElement* e, int depth, const Model& model, const tinyxml2::XMLElement* commentsFrom) {
    const std::string indent(static_cast<std::size_t>(depth) * 4, ' ');
    out << indent << '<' << e->Name();
    for(auto* a = e->FirstAttribute(); a; a = a->Next()) {
        out << ' ' << a->Name() << "=\"" << escape(a->Value(), true) << '"';
    }

    std::vector<const tinyxml2::XMLComment*> trailing;
    std::vector<Item> children = items(e, trailing);
    const bool isModel = e == model.replaces;
    const bool isEntry = e->Parent() && e->Parent()->ToElement() && std::string(e->Parent()->ToElement()->Name()) == "TreeNodesModel";
    if(isModel) {
        children = model.entries;
        trailing = model.trailing;
    } else if(isEntry) {
        if(commentsFrom) {
            // a regenerated entry keeps the old one's comments on the same ports
            std::vector<Item> old = items(commentsFrom, trailing);
            for(auto& child : children) {
                for(const auto& o : old) {
                    if(sortKey(o.element) == sortKey(child.element)) {
                        child.comments = o.comments;
                    }
                }
            }
        }
        sortItems(children);
    }

    std::string text = e->GetText() ? e->GetText() : "";
    const auto first = text.find_first_not_of(" \t\r\n");
    text = first == std::string::npos ? "" : text.substr(first, text.find_last_not_of(" \t\r\n") - first + 1);
    if(children.empty() && trailing.empty()) {
        if(text.empty()) {
            out << "/>\n";
        } else {
            out << '>' << escape(text, false) << "</" << e->Name() << ">\n";
        }
        return;
    }
    out << ">\n";
    writeItems(out, children, trailing, depth + 1, model);
    out << indent << "</" << e->Name() << ">\n";
}

// The whole document; comments outside the root are kept where they are.
std::string writeDocument(const tinyxml2::XMLDocument& doc, const Model& model) {
    std::ostringstream out;
    out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    for(auto* n = doc.FirstChild(); n; n = n->NextSibling()) {
        if(auto* c = n->ToComment()) {
            writeComment(out, c, 0);
        } else if(auto* e = n->ToElement()) {
            write(out, e, 0, model);
        }
    }
    return out.str();
}

const tinyxml2::XMLElement* modelOf(const tinyxml2::XMLDocument& doc) {
    return doc.RootElement() ? doc.RootElement()->FirstChildElement("TreeNodesModel") : nullptr;
}

void parseModel(tinyxml2::XMLDocument& doc, const std::string& text, const std::string& what) {
    if(doc.Parse(text.c_str()) != tinyxml2::XML_SUCCESS || !modelOf(doc)) {
        throw std::runtime_error(what + " is not a node model file");
    }
}

// The existing file with its plugin entries replaced by the generated ones. SubTree entries (notgroot's), every
// comment and the rest of the file are kept; an entry's comments go with it, and go if it does.
std::string generate(const std::string& existingText) {
    auto factory = std::make_shared<BT::BehaviorTreeFactory>();
    registerPluginsForFactory(factory, AUTONOMY_PKG_NAME);

    tinyxml2::XMLDocument generated, existing;
    parseModel(generated, BT::writeTreeNodesModelXML(*factory, false), "BehaviorTree.CPP's model");
    parseModel(existing, existingText.empty() ? "<root BTCPP_format=\"4\"><TreeNodesModel/></root>" : existingText, "the existing file");

    Model model{modelOf(existing), {}, {}};
    std::vector<const tinyxml2::XMLComment*> unused;
    const std::vector<Item> nodes = items(modelOf(generated), unused);
    std::map<std::string, Item> old;
    for(const auto& item : items(model.replaces, model.trailing)) {
        old[sortKey(item.element)] = item;
    }
    for(const auto& [id, item] : old) {
        const bool generatedToo = std::any_of(nodes.begin(), nodes.end(), [&](const Item& n) { return sortKey(n.element) == id; });
        if(std::string(item.element->Name()) == "SubTree" && !generatedToo) {
            model.entries.push_back(item);
        }
    }
    for(const auto& item : nodes) {
        const auto found = old.find(sortKey(item.element));
        if(found != old.end()) {
            model.entries.push_back({found->second.comments, item.element, found->second.element});
        } else {
            model.entries.push_back(item);
        }
    }
    sortItems(model.entries);
    return writeDocument(existing, model);
}

// SubTree ports that carry default= (notgroot docs/contract.md section 3 bans them), as "Subtree port"
std::vector<std::string> subtreeDefaults(const std::string& text) {
    std::vector<std::string> found;
    tinyxml2::XMLDocument doc;
    if(doc.Parse(text.c_str()) != tinyxml2::XML_SUCCESS || !modelOf(doc)) {
        return found;
    }
    for(auto* e = modelOf(doc)->FirstChildElement("SubTree"); e; e = e->NextSiblingElement("SubTree")) {
        for(auto* port = e->FirstChildElement(); port; port = port->NextSiblingElement()) {
            if(port->Attribute("default")) {
                found.push_back(std::string(e->Attribute("ID") ? e->Attribute("ID") : "") + " " + (port->Attribute("name") ? port->Attribute("name") : ""));
            }
        }
    }
    return found;
}

// BehaviorTree.CPP's own nodes only, for the editor (notgroot's content/builtin_models.xml)
std::string generateBuiltins() {
    BT::BehaviorTreeFactory factory;
    tinyxml2::XMLDocument generated, doc;
    parseModel(generated, BT::writeTreeNodesModelXML(factory, true), "BehaviorTree.CPP's model");

    const std::string header = std::string(" BehaviorTree.CPP ") + BT::LibraryVersionString() +
        " builtin node models: ros2 run riptide_autonomy2 updateNodeModels --builtins FILE ";
    parseModel(doc, "<root BTCPP_format=\"4\"><!--" + header + "--><TreeNodesModel/></root>", "the builtin template");

    Model model{modelOf(doc), {}, {}};
    std::vector<const tinyxml2::XMLComment*> unused;
    for(const auto& item : items(modelOf(generated), unused)) {
        if(factory.builtinNodes().count(sortKey(item.element))) {
            model.entries.push_back(item);
        }
    }
    sortItems(model.entries);
    return writeDocument(doc, model);
}
}

int main(int argc, char** argv) {
    bool check = false, builtins = false, fileGiven = false;
    std::filesystem::path file = std::filesystem::path(AUTONOMY_TREE_DIR) / "lib" / "node_models.xml";
    for(int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if(arg == "--check") {
            check = true;
        } else if(arg == "--builtins") {
            builtins = true;
        } else if(arg == "-h" || arg == "--help") {
            std::cout << "usage: updateNodeModels [--check] [node_models.xml]\n"
                         "       updateNodeModels [--check] --builtins FILE\n";
            return 0;
        } else {
            file = arg;
            fileGiven = true;
        }
    }

    // the builtins are not the repository's node list, so never default to writing over it
    if(builtins && !fileGiven) {
        std::cerr << "updateNodeModels: --builtins needs a FILE\n";
        return 2;
    }

    std::string existing;
    if(std::ifstream in{file}) {
        std::stringstream buffer;
        buffer << in.rdbuf();
        existing = buffer.str();
    }

    try {
        const std::string text = builtins ? generateBuiltins() : generate(existing);

        const auto defaults = builtins ? std::vector<std::string>() : subtreeDefaults(text);
        for(const auto& port : defaults) {
            std::cerr << file.string() << ": SubTree port " << port << " has default= (the contract allows none)\n";
        }
        if(check && !defaults.empty()) {
            return 1;
        }

        if(text == existing) {
            std::cout << file.string() << " is up to date\n";
            return 0;
        }
        if(check) {
            std::cerr << file.string() << " is out of date: run updateNodeModels\n";
            return 1;
        }
        std::filesystem::create_directories(file.parent_path());
        std::ofstream out(file);
        out << text;
        out.close();
        if(out.fail()) {
            throw std::runtime_error("cannot write " + file.string());
        }
        std::cout << "wrote " << file.string() << '\n';
        return 0;
    } catch(const std::exception& e) {
        std::cerr << "updateNodeModels: " << e.what() << '\n';
        return 2;
    }
}
