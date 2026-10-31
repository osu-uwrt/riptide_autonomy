#include "autonomy_test/autonomy_testing.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <filesystem>

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
                factory->createTreeFromFile(entry.path().string());
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
                factory->createTree(entry.path().stem().string());
            } catch (const std::exception& e) {
                ADD_FAILURE() << entry.path().filename() << ": " << e.what();
                throw;
            }
        }) << entry.path().filename();
        ++loaded;
    }
    EXPECT_GT(loaded, 0);
}
