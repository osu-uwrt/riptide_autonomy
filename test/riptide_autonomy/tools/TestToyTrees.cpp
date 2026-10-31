#include "autonomy_test/autonomy_testing.hpp"

#include <filesystem>
#include <fstream>
#include <regex>

#include <unistd.h>

// Small trees for what the real missions rely on: includes, subtree remaps, number text into subtrees, the builtin
// decorators and shared ROS entities.

namespace {

class ToyTrees : public BtTest {
    protected:
    void SetUp() override {
        BtTest::SetUp();
        factory = std::make_shared<BT::BehaviorTreeFactory>();
        registerPluginsForFactory(factory, "riptide_autonomy2");
        dir = std::filesystem::temp_directory_path() / ("riptide_toy_trees_" + std::to_string(getpid()));
        std::filesystem::create_directories(dir / "lib");
    }

    void TearDown() override {
        std::filesystem::remove_all(dir);
        factory.reset();
        BtTest::TearDown();
    }

    void writeFile(const std::string& name, const std::string& body) {
        std::ofstream(dir / name) << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n" << body;
    }

    std::string text(BT::Tree& tree, const std::string& key) {
        return tree.rootBlackboard()->get<std::string>(key);
    }

    std::shared_ptr<BT::BehaviorTreeFactory> factory;
    std::filesystem::path dir;
};

}

// lib/A and lib/B both include C: each file loads once and the mission runs, as in the lib layout
TEST_F(ToyTrees, test_diamond_includes) {
    writeFile("Mission.xml", R"(<root BTCPP_format="4" main_tree_to_execute="Mission">
    <include path="lib/A.xml"/>
    <include path="lib/B.xml"/>
    <BehaviorTree ID="Mission">
        <Sequence>
            <SubTree ID="A" out="{a}"/>
            <SubTree ID="B" out="{b}"/>
        </Sequence>
    </BehaviorTree>
</root>
)");
    writeFile("lib/A.xml", R"(<root BTCPP_format="4">
    <include path="C.xml"/>
    <BehaviorTree ID="A">
        <SubTree ID="C" text="from A" out="{out}"/>
    </BehaviorTree>
</root>
)");
    writeFile("lib/B.xml", R"(<root BTCPP_format="4">
    <include path="C.xml"/>
    <BehaviorTree ID="B">
        <SubTree ID="C" text="from B" out="{out}"/>
    </BehaviorTree>
</root>
)");
    writeFile("lib/C.xml", R"(<root BTCPP_format="4">
    <BehaviorTree ID="C">
        <SetBlackboardV3 output_key="out" value="{text}"/>
    </BehaviorTree>
</root>
)");

    // from another folder, as doTask runs: includes resolve against the including file
    const auto cwd = std::filesystem::current_path();
    std::filesystem::current_path(std::filesystem::temp_directory_path());
    BT::Tree tree = factory->createTreeFromFile(dir / "Mission.xml");
    std::filesystem::current_path(cwd);

    ASSERT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
    EXPECT_EQ(text(tree, "a"), "from A");
    EXPECT_EQ(text(tree, "b"), "from B");
    EXPECT_EQ(tree.subtrees.size(), 5u); // Mission, A, B and a C for each
}

// a SubTree argument is the parent's key in braces, anything else is text; subtrees don't see the parent's keys
TEST_F(ToyTrees, test_subtree_remap_literal_vs_key) {
    BT::Tree tree = factory->createTreeFromText(R"(<root BTCPP_format="4" main_tree_to_execute="Main">
    <BehaviorTree ID="Main">
        <Sequence>
            <SetBlackboardV3 output_key="parent_key" value="world"/>
            <SubTree ID="Echo" msg="hello" result="{literal}"/>
            <SubTree ID="Echo" msg="{parent_key}" result="{remapped}"/>
            <SubTree ID="Peek" result="{peeked}"/>
        </Sequence>
    </BehaviorTree>
    <BehaviorTree ID="Echo">
        <SetBlackboardV3 output_key="result" value="{msg}"/>
    </BehaviorTree>
    <BehaviorTree ID="Peek">
        <SetBlackboardV3 output_key="result" value="{parent_key}"/>
    </BehaviorTree>
</root>
)");

    ASSERT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
    EXPECT_EQ(text(tree, "literal"), "hello");
    EXPECT_EQ(text(tree, "remapped"), "world");
    // the subtree's own parent_key was never set, so it copies nothing (like {__v3_empty} in converted trees)
    const auto peeked = tree.rootBlackboard()->getEntry("peeked");
    EXPECT_TRUE(!peeked || peeked->value.empty());
}

// BT.CPP 4.10 types subtree text by its looks: "2" makes the subtree's key an int. UWRT inputs and SetBlackboardV3
// take any type, so numbers go to subtrees as plain text; a std::string port (Info's message) still refuses them.
TEST_F(ToyTrees, test_subtree_number_literal) {
    const std::string xml = R"(<root BTCPP_format="4" main_tree_to_execute="Main">
    <BehaviorTree ID="Main">
        <SubTree ID="Numbers" n="2" d="0.5" out="{out}"/>
    </BehaviorTree>
    <BehaviorTree ID="Numbers">
        <Sequence>
            <CompareNums a="{n}" b="2" test="=="/>
            <SetBlackboardV3 output_key="out" value="{d}"/>
            READER
        </Sequence>
    </BehaviorTree>
</root>
)";
    auto withReader = [&xml](const std::string& reader) {
        return std::regex_replace(xml, std::regex("READER"), reader);
    };
    BT::Tree tree = factory->createTreeFromText(withReader("<AlwaysSuccess/>"));
    initRosForTree(tree, toolNode);
    ASSERT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
    EXPECT_DOUBLE_EQ(tree.rootBlackboard()->get<double>("out"), 0.5);

    EXPECT_THROW((void)factory->createTreeFromText(withReader("<Info message=\"{n}\"/>")), BT::RuntimeError);
}

// the builtin decorators the trees use: Repeat and RetryUntilSuccessful return RUNNING between iterations of a child
// that finishes in one tick (doTask ticks once per loop), and Precondition compares with a script
TEST_F(ToyTrees, test_builtin_decorators) {
    BT::Tree tree = factory->createTreeFromText(R"(<root BTCPP_format="4" main_tree_to_execute="Main">
    <BehaviorTree ID="Main">
        <Sequence>
            <Script code="repeats := 0; retries := 0"/>
            <Repeat num_cycles="3">
                <Script code="repeats := repeats + 1"/>
            </Repeat>
            <ForceSuccess>
                <RetryUntilSuccessful num_attempts="4">
                    <Sequence>
                        <Script code="retries := retries + 1"/>
                        <AlwaysFailure/>
                    </Sequence>
                </RetryUntilSuccessful>
            </ForceSuccess>
            <SetBlackboardV3 output_key="a" value="x"/>
            <SetBlackboardV3 output_key="copy" value="{a}"/>
            <Precondition if="copy == 'x'" else="FAILURE">
                <SetBlackboardV3 output_key="matched" value="yes"/>
            </Precondition>
            <Inverter>
                <Precondition if="copy == 'y'" else="FAILURE">
                    <AlwaysSuccess/>
                </Precondition>
            </Inverter>
        </Sequence>
    </BehaviorTree>
</root>
)");

    int ticks = 1;
    BT::NodeStatus status = tree.tickExactlyOnce();
    while(status == BT::NodeStatus::RUNNING && ticks < 100) {
        status = tree.tickExactlyOnce();
        ticks++;
    }
    ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
    EXPECT_EQ(ticks, 6); // one, then RUNNING after 2 repeats and 3 retries
    EXPECT_EQ(tree.rootBlackboard()->get<int>("repeats"), 3);
    EXPECT_EQ(tree.rootBlackboard()->get<int>("retries"), 4);
    EXPECT_EQ(text(tree, "matched"), "yes");
}

// many GetOdometry copies share ONE subscription, listen only once ticked, and each waits for a message newer
// than its start
TEST_F(ToyTrees, test_many_get_odometry_share_one_subscription) {
    const int count = 50;
    std::string nodes;
    for(int i = 0; i < count; i++) {
        nodes += "            <GetOdometry x=\"{x" + std::to_string(i) + "}\"/>\n";
    }
    BT::Tree tree = factory->createTreeFromText(R"(<root BTCPP_format="4" main_tree_to_execute="Main">
    <BehaviorTree ID="Main">
        <Parallel success_count="-1" failure_count="1">
)" + nodes + R"(        </Parallel>
    </BehaviorTree>
</root>
)");
    initRosForTree(tree, toolNode);

    auto source = RosEntityCache::subscription<nav_msgs::msg::Odometry>(toolNode, ODOMETRY_TOPIC, 10);
    EXPECT_EQ(RosEntityCache::liveTotal(), 1u);
    EXPECT_EQ(source->listenerCount(), 0u); // nothing ticked yet

    auto pub = toolNode->create_publisher<nav_msgs::msg::Odometry>(ODOMETRY_TOPIC, 10);
    for(int i = 0; i < 300 && pub->get_subscription_count() == 0; i++) {
        toolNode->spinForTime(10ms);
    }
    EXPECT_EQ(pub->get_subscription_count(), 1u); // one subscription in the ROS graph, not 50

    // a message from before the nodes start: kept as the latest, and ignored by them
    nav_msgs::msg::Odometry odom;
    odom.pose.pose.orientation.w = 1;
    odom.pose.pose.position.x = 1;
    for(int i = 0; i < 100 && !source->latest(); i++) {
        pub->publish(odom);
        toolNode->spinForTime(20ms);
    }
    ASSERT_TRUE(source->latest());
    EXPECT_DOUBLE_EQ(source->latest()->pose.pose.position.x, 1);

    ASSERT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
    EXPECT_EQ(source->listenerCount(), static_cast<size_t>(count));

    odom.pose.pose.position.x = 2;
    const auto start = std::chrono::steady_clock::now();
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while(status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() - start < 3s) {
        pub->publish(odom);
        toolNode->spinForTime(20ms);
        status = tree.tickExactlyOnce();
    }
    ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
    for(int i = 0; i < count; i++) {
        EXPECT_EQ(text(tree, "x" + std::to_string(i)), "2") << "GetOdometry " << i;
    }
    EXPECT_EQ(RosEntityCache::liveTotal(), 1u);
}

// shared entities go away with the last tree holding them
TEST_F(ToyTrees, test_entities_released_with_tree) {
    {
        BT::Tree tree = factory->createTreeFromText(R"(<root BTCPP_format="4" main_tree_to_execute="Main">
    <BehaviorTree ID="Main">
        <Sequence>
            <PublishToController mode="1" x="0" y="0" z="0"/>
            <PublishToController isOrientation="true" mode="1" x="0" y="0" z="0"/>
            <SetStatus status="moving"/>
            <SetStatus status="searching"/>
            <GetOdometry/>
            <GetOdometry/>
        </Sequence>
    </BehaviorTree>
</root>
)");
        initRosForTree(tree, toolNode);
        auto counts = RosEntityCache::liveCounts();
        EXPECT_EQ(counts["pub"], 3u); // controller linear and angular, LEDs
        EXPECT_EQ(counts["sub"], 1u); // odometry
        EXPECT_EQ(RosEntityCache::liveTotal(), 4u);
    }
    EXPECT_EQ(RosEntityCache::liveTotal(), 0u);
}
