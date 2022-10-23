#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <riptide_msgs2/action/execute_tree.hpp>
#include <riptide_msgs2/srv/list_trees.hpp>

#include <vector>
#include <chrono>
#include <filesystem>

#include "riptide_autonomy/autonomy_lib.hpp"
#include "riptide_autonomy/UWRTLogger.hpp"

/**
 * C++ Script that runs a given behavior tree.
 * Script will take in the file path to a behavior
 * tree XML file as an argument, then run that
 * tree and return the result (0 if success, 1 if failure)
 */
#ifndef AUTONOMY_PKG_NAME
#define AUTONOMY_PKG_NAME "ritpide_autonomy2"
#endif
#define AUTONOMY_HOME_DIR "/osu-uwrt/riptide_software/src/riptide_autonomy/trees"

using namespace BT;
using namespace std::chrono_literals;

std::string getEnvVar(const char *name)
{
    const char *env = std::getenv(name);
    if (env == nullptr)
    {
        RCLCPP_INFO(log, "DoTask: %s environment variable not found!", name);
        return "";
    }

    return std::string(env);
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
            declare_parameter<bool>("enable_zmq", false);
            declare_parameter<bool>("enable_cout", true);
            declare_parameter<std::string>("cout_file", getEnvVar("HOME") + "/osu-uwrt/autonomy_log.txt");
            declare_parameter<std::vector<std::string>>("ext_plugin_list", std::vector<std::string>());
            declare_parameter<std::vector<std::string>>("ext_tree_dirs", std::vector<std::string>());

            // load the params
            try
            {
                RCLCPP_INFO(this->get_logger(), "Getting parameter data");
                // get param values
                enableZMQ = get_parameter("enable_zmq").as_bool();
                enableCout = get_parameter("enable_cout").as_bool();
                treeDirs = get_parameter("ext_tree_dirs").as_string_array();
                pluginPaths = get_parameter("ext_plugin_list").as_string_array();
                coutFilePath = get_parameter("cout_file").as_string();
            }
            catch (const std::exception &e)
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
            RCLCPP_INFO(this->get_logger(), "Registering autonomy core plugin");
            registerPluginsForFactory(factory, AUTONOMY_PKG_NAME);

            // load other plugins from the paramter server
            for (auto plugin : pluginPaths)
            {
                try{
                    RCLCPP_INFO_STREAM(this->get_logger(), "Registering additional plugin: " << plugin);
                    factory->registerFromPlugin(plugin);
                }
                catch(BT::RuntimeError & e){
                    RCLCPP_ERROR_STREAM(get_logger(), "Could not load plugin: " << e.what());
                }
                
            }

            // automatically add osu-uwrt riptide autonomy and the ament index dir
            treeDirs.push_back(getEnvVar("HOME") + AUTONOMY_HOME_DIR);
            treeDirs.push_back(ament_index_cpp::get_package_share_directory(AUTONOMY_PKG_NAME) + "/trees");
        }

        rclcpp_action::GoalResponse handleGoal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const ExecuteTree::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Received goal request with tree name %s", goal->tree.c_str());
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
            // prepare the result message
            riptide_msgs2::action::ExecuteTree::Result::SharedPtr result =
                std::make_shared<riptide_msgs2::action::ExecuteTree::Result>();

            try
            {
                // load the tree file contents in to a BT context
                Tree tree = factory->createTreeFromFile(goal_handle->get_goal()->tree);
                initRosForTree(tree, this->shared_from_this());

                // add the loggers to the BT context
                RCLCPP_INFO(log, "DoTask: Loading Monitor");
                PublisherZMQ zmq(tree); // publishes behaviortree data to a groot in real time
                FileLogger fileLogger(tree, coutFilePath.c_str());
                // StdCoutLogger coutLogger(tree);
                UwrtLogger uwrtLogger(tree, this->shared_from_this());

                // configure our loggers
                // coutLogger.setEnabled(enableCout);
                zmq.setEnabled(enableZMQ);
                uwrtLogger.setEnabled(true);

                // set up idle sleep rate
                rclcpp::Rate loop_rate(10ms);

                // start ticking the tree with feedback
                // keep executing tick until it returns either SUCCESS or FAILURE
                auto tickStatus = NodeStatus::RUNNING;
                while (tickStatus == NodeStatus::RUNNING)
                {
                    // always gets ticked once
                    tickStatus = tree.tickRoot();

                    // check for a cancel
                    if (goal_handle->is_canceling())
                    {
                        result->returncode = 0;
                        goal_handle->canceled(result);
                        RCLCPP_INFO(log, "DoTask: Cancelled current action goal");
                        return;
                    }

                    // sleep a bit while we wait
                    loop_rate.sleep();
                }

                fileLogger.flush();
                // coutLogger.flush(); // will flush if enabled

                std::string resultStr = "SUCCESS";
                switch(tickStatus) {
                    case NodeStatus::FAILURE:
                        resultStr = "FAILURE";
                        break;
                    case NodeStatus::IDLE:
                        resultStr = "IDLE";
                        break;
                    case NodeStatus::RUNNING:
                        resultStr = "RUNNING";
                        break;
                    default:
                        resultStr = "SUCCESS";
                        break;
                }

                RCLCPP_INFO(log, "Tree ended with status %s", resultStr.c_str());

                // wrap this party up and finish execution
                result->returncode = (int)tickStatus;
                goal_handle->succeed(result);

                // bail early, all other code is error checking
                return;
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR_STREAM(log, "Error occurred while ticking tree. Aborting tree! Error: " << e.what());
            }
            catch (...)
            {
                RCLCPP_ERROR(log, "Unknown error while ticking tree. Aborting tree!");
            }
            // if error, abort
            result->returncode = -1;
            goal_handle->abort(result);
        }

        void handleService(const riptide_msgs2::srv::ListTrees::Request::SharedPtr request,
                           riptide_msgs2::srv::ListTrees::Response::SharedPtr response)
        {
            (void)request; // empty request

            std::vector<std::string> treeFiles;

            // iterate all the search dirs
            for (auto dir : treeDirs)
            {
                // iterate the files in the search dirs and test them to see if they are xml
                for (const auto &entry : std::filesystem::directory_iterator(dir))
                {
                    std::string file = std::string(entry.path());
                    if (file.find(".xml") != std::string::npos)
                        treeFiles.push_back(file);
                }
            }

            // hand off the listing of possible files
            response->trees = treeFiles;
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
        std::vector<std::string> treeDirs;
        std::vector<std::string> pluginPaths;

        // cout log file location
        std::string coutFilePath;
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