#include "autonomy_test/autonomy_testing.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tinyxml2.h>

#include <filesystem>

namespace {

std::filesystem::path treeDir() {
    return std::filesystem::path(ament_index_cpp::get_package_share_directory("riptide_autonomy2")) / "trees";
}

// the missions: every .xml directly in trees/
std::vector<std::filesystem::path> missions() {
    std::vector<std::filesystem::path> result;
    for (const auto& entry : std::filesystem::directory_iterator(treeDir())) {
        if (entry.is_regular_file() && entry.path().extension() == ".xml") {
            result.push_back(entry.path());
        }
    }
    std::sort(result.begin(), result.end());
    return result;
}

// IDs of the file's <BehaviorTree>s
std::vector<std::string> treeIds(const tinyxml2::XMLElement* root) {
    std::vector<std::string> ids;
    for (auto* tree = root->FirstChildElement("BehaviorTree"); tree; tree = tree->NextSiblingElement("BehaviorTree")) {
        ids.push_back(tree->Attribute("ID") ? tree->Attribute("ID") : "");
    }
    return ids;
}

}

// Every installed tree must build with the registered nodes: catches unknown node
// IDs, bad port names and broken subtree remappings before a tree is run.
TEST_F(BtTest, test_all_trees_load) {
    auto factory = std::make_shared<BT::BehaviorTreeFactory>();
    registerPluginsForFactory(factory, "riptide_autonomy2");
    const std::filesystem::path trees =
        std::filesystem::path(ament_index_cpp::get_package_share_directory("riptide_autonomy2")) / "trees";
    int loaded = 0;
    for (const auto& entry : std::filesystem::directory_iterator(trees)) {
        if (entry.path().extension() != ".xml") {
            continue;
        }
        EXPECT_NO_THROW({
            try {
                factory->clearRegisteredBehaviorTrees(); // one factory for every file, as doTask uses it
                (void)factory->createTreeFromFile(entry.path().string());
            } catch (const std::exception& e) {
                ADD_FAILURE() << entry.path().filename() << ": " << e.what();
                throw;
            }
        }) << entry.path().filename();
        ++loaded;
    }
    EXPECT_GT(loaded, 0);
}

// Every shared subtree in trees/lib builds on its own: its file includes everything it uses.
TEST_F(BtTest, test_all_lib_subtrees_load) {
    auto factory = std::make_shared<BT::BehaviorTreeFactory>();
    registerPluginsForFactory(factory, "riptide_autonomy2");
    const std::filesystem::path lib =
        std::filesystem::path(ament_index_cpp::get_package_share_directory("riptide_autonomy2")) / "trees" / "lib";
    int loaded = 0;
    for (const auto& entry : std::filesystem::directory_iterator(lib)) {
        // node_models.xml is the editor's node list, not a tree.
        if (entry.path().extension() != ".xml" || entry.path().filename() == "node_models.xml") {
            continue;
        }
        EXPECT_NO_THROW({
            try {
                factory->clearRegisteredBehaviorTrees();
                factory->registerBehaviorTreeFromFile(entry.path().string());
                (void)factory->createTree(entry.path().stem().string());
            } catch (const std::exception& e) {
                ADD_FAILURE() << entry.path().filename() << ": " << e.what();
                throw;
            }
        }) << entry.path().filename();
        ++loaded;
    }
    EXPECT_GT(loaded, 0);
}

// The file layout the editor relies on (notgroot docs/contract.md section 1): a mission has one tree, its
// main_tree_to_execute; lib/<ID>.xml has one tree, ID; only lib/ files are included.
TEST_F(BtTest, test_tree_layout) {
    ASSERT_FALSE(missions().empty());
    for (const auto& mission : missions()) {
        tinyxml2::XMLDocument doc;
        ASSERT_EQ(doc.LoadFile(mission.c_str()), tinyxml2::XML_SUCCESS) << mission.filename();
        const char* main = doc.RootElement()->Attribute("main_tree_to_execute");
        ASSERT_NE(main, nullptr) << mission.filename();
        EXPECT_EQ(treeIds(doc.RootElement()), std::vector<std::string>{main}) << mission.filename();
        for (auto* include = doc.RootElement()->FirstChildElement("include"); include; include = include->NextSiblingElement("include")) {
            EXPECT_EQ(std::string(include->Attribute("path")).rfind("lib/", 0), 0u) << mission.filename();
        }
    }

    for (const auto& entry : std::filesystem::directory_iterator(treeDir() / "lib")) {
        if (entry.path().extension() != ".xml" || entry.path().filename() == "node_models.xml") {
            continue;
        }
        tinyxml2::XMLDocument doc;
        ASSERT_EQ(doc.LoadFile(entry.path().c_str()), tinyxml2::XML_SUCCESS) << entry.path().filename();
        EXPECT_EQ(treeIds(doc.RootElement()), std::vector<std::string>{entry.path().stem().string()}) << entry.path().filename();
    }
}

// Every mission's ROS entities are shared: a handful per mission, not one per node copy as before
TEST_F(BtTest, test_missions_share_ros_entities) {
    auto factory = std::make_shared<BT::BehaviorTreeFactory>();
    registerPluginsForFactory(factory, "riptide_autonomy2");
    for (const auto& mission : missions()) {
        BT::Tree tree = factory->createTreeFromFile(mission);
        initRosForTree(tree, toolNode);

        size_t rosNodes = 0;
        tree.applyVisitor([&rosNodes](BT::TreeNode* node) {
            rosNodes += dynamic_cast<UwrtBtNode*>(node) != nullptr;
        });
        EXPECT_LE(RosEntityCache::liveTotal(), 20u) << mission.filename() << " (" << rosNodes << " ROS nodes)";
    }
    EXPECT_EQ(RosEntityCache::liveTotal(), 0u);
}
