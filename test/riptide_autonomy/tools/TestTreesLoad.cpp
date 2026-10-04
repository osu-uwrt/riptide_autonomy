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
