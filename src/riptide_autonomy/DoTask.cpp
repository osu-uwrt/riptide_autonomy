#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <riptide_msgs2/action/execute_tree.hpp>
#include <riptide_msgs2/srv/list_trees.hpp>

#include <vector>

#include "UwrtBtNode.hpp"

/**
 * C++ Script that runs a given behavior tree.
 * Script will take in the file path to a behavior
 * tree XML file as an argument, then run that
 * tree and return the result (0 if success, 1 if failure)
 */

// define logger for RCLCPP_INFO, RCLCPP_WARN, and RCLCPP_ERROR
#define log rclcpp::get_logger("autonomy")

using namespace BT;

const char *AUTONOMY_PATH_FROM_HOME = "/osu-uwrt/riptide_software/src/riptide_autonomy/";

std::string getEnvironmentVariable(const char *name)
{
    const char *env = std::getenv(name);
    if (env == nullptr)
    {
        RCLCPP_INFO(log, "DoTask: %s environment variable not found!", name);
        return "";
    }

    return env;
}

namespace do_task
{
    using namespace std::placeholders;
    using ExecuteTree = riptide_msgs2::action::ExecuteTree;
    using GoalHandleExecuteTree = rclcpp_action::ServerGoalHandle<ExecuteTree>;

    class BTExecutor : public rclcpp::Node
    {
    public:
        BTExecutor() : Node("autonomy_dotask")
        {
            // make an action server for running the autonomy trees
            actionServer = rclcpp_action::create_server<ExecuteTree>(
                this,
                "autonomy/run_tree",
                std::bind(&BTExecutor::handleGoal, this, _1, _2),
                std::bind(&BTExecutor::handleCancel, this, _1),
                std::bind(&BTExecutor::handleAccepted, this, _1));

            // make a service server for listing all of the trees loaded / availiable
            listTreeServer = create_service<riptide_msgs2::srv::ListTrees>(
                "autonomy/list_trees",
                std::bind(&BTExecutor::handleService, this, _1, _2));

            // declare params
            declare_parameter<bool>("enable_zmq", true);
            declare_parameter<bool>("enbale_cout", true);
            declare_parameter<std::vector<std::string>>("ext_plugin_list", {""});
            declare_parameter<std::vector<std::string>>("ext_tree_dirs", {""});

            // load the params
            try
            {
                RCLCPP_INFO(this->get_logger(), "Getting parameter data");
                // get param values
                enableZMQ = get_parameter("enable_zmq").as_bool();
                enableCout = get_parameter("enable_cout").as_bool();
                auto treeDirs = get_parameter("ext_tree_dirs").as_string_array();
                pluginPaths = get_parameter("ext_plugin_list").as_string_array();
            }
            catch (std::exception e)
            {
                RCLCPP_ERROR_STREAM(this->get_logger(),
                                    "Error checking params: " << e.what());
            }
            catch (...)
            {
                RCLCPP_ERROR(this->get_logger(), "Unknown error checking params");
            }

            // create the behavior tree factory context
            factory = std::make_shared<BehaviorTreeFactory>();

            // load our plugins from ament index
            std::string amentIndexPath = ""; // TODO Make this work to scan ament index and get to our plugin
            factory->registerFromPlugin(amentIndexPath);

            // load other plugins from the paramter server
            RCLCPP_INFO(this->get_logger(), "Registering additional plugins");
            for(auto plugin : pluginPaths){
                factory->registerFromPlugin(plugin);
            }

            // create file listing
            treeFiles = std::vector<std::string>();

            // scan for available trees in paramters and in osu-uwrt dir
            RCLCPP_INFO(this->get_logger(), "Scanning for availiable trees");
        }

        rclcpp_action::GoalResponse handleGoal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const ExecuteTree::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Received goal request with tree name %s", goal->tree);
            (void)uuid;

            // test if the tree is running
            if (executionThread.joinable())
            {
                // tree is running, so we cannot accept another
                return rclcpp_action::GoalResponse::REJECT;
            }

            // test if the tree exists

            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handleCancel(
            const std::shared_ptr<GoalHandleExecuteTree> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel BehaviorTree");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handleAccepted(const std::shared_ptr<GoalHandleExecuteTree> goal_handle)
        {
            // this needs to return quickly to avoid blocking the executor, so spin up a new thread
            executionThread = std::thread{std::bind(&BTExecutor::execute, this, _1), goal_handle};
            executionThread.detach();
        }

        void execute(const std::shared_ptr<GoalHandleExecuteTree> goal_handle)
        {
            // do the thing for the running task in here and send feedback!

            // load the tree file contents in to a BT context
            Tree tree = factory->createTreeFromFile("treePath");

            // give each BT node access to our RCLCPP context
            for (auto &node : tree.nodes)
            {
                // Not a typo: it is "=", not "=="
                if (auto btNode = dynamic_cast<UwrtBtNode *>(node.get()))
                {
                    btNode->init(this->shared_from_this());
                }
            }

            // add the loggers to the BT context
            RCLCPP_INFO(log, "DoTask: Loading Monitor");
            PublisherZMQ zmq(tree); // publishes behaviortree data to a groot in real time
            FileLogger fileLogger(tree, "logFile.c_str()");
            StdCoutLogger coutLogger(tree);

            // test if node COUT logger is enabled
            bool coutEnabled = true; // = indexOfStr(argv, "--log-cout", argc) > -1;
            coutLogger.setEnabled(coutEnabled);
            if (coutEnabled)
                RCLCPP_INFO(log, "DoTask: Cout Logging Enabled.");

            // start ticking the tree with feedback
            NodeStatus result = tree.tickRoot();
            fileLogger.flush();
            coutLogger.flush(); // will flush if enabled

            // RCLCPP_INFO(log, "Tree returned with %s", (result == NodeStatus::SUCCESS ? "SUCCESS" : "FAILED"));

            // if interrupted by cancel, shut down and respond with cancelled

            // otherwise continue untill error or complete

            // if error, abort
        }

        void handleService(const riptide_msgs2::srv::ListTrees::Request::SharedPtr request,
                           riptide_msgs2::srv::ListTrees::Response::SharedPtr response)
        {
        }

    private:
        // ros action and service servers
        rclcpp_action::Server<ExecuteTree>::SharedPtr actionServer;
        rclcpp::Service<riptide_msgs2::srv::ListTrees>::SharedPtr listTreeServer;

        // logger enablement flags
        bool enableZMQ, enableCout;

        // execution context thread for the action server
        std::thread executionThread;

        // behavior tree factory context
        std::shared_ptr<BehaviorTreeFactory> factory;

        // full tree file path vector to load
        std::vector<std::string> treeFiles;
        std::vector<std::string> pluginPaths;
    };
} // namespace do_task

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // create our node context
    auto node = std::make_shared<do_task::BTExecutor>();

    rclcpp::spin(node);

    rclcpp::shutdown();
}