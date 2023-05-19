#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <riptide_msgs2/action/execute_tree.hpp>
#include <riptide_msgs2/srv/list_trees.hpp>

#include <vector>
#include <chrono>
#include <filesystem>

#include <unistd.h>

#include "riptide_autonomy/autonomy_lib.hpp"
#include "riptide_autonomy/UWRTLogger.hpp"

/**
 * ROS2 action server that runs behavior trees.
 * Call autonomy/run_tree with the riptide_msgs2/msg/RunTree command
 * Can use the autonomy/list_trees service to list out trees in the package
 */
#ifndef AUTONOMY_PKG_NAME
#define AUTONOMY_PKG_NAME "riptide_autonomy2"
#endif
#define AUTONOMY_TREE_DIR \
    std::string(__FILE__).substr(0, std::string(__FILE__).find("/riptide_autonomy/")) + std::string("/riptide_autonomy/trees")

using namespace BT;
using namespace std::chrono_literals;

namespace do_task
{
    using namespace std::placeholders;
    using ExecuteTree = riptide_msgs2::action::ExecuteTree;
    using GoalHandleExecuteTree = rclcpp_action::ServerGoalHandle<ExecuteTree>;
    using LedCmd = riptide_msgs2::msg::LedCommand;

    class BTExecutor : public rclcpp::Node
    {
    public:
        BTExecutor() : Node("autonomy_dotask")
        {
            // create publishers
            linearPub = this->create_publisher<riptide_msgs2::msg::ControllerCommand>(CONTROL_LINEAR_TOPIC, 10);
            angularPub = this->create_publisher<riptide_msgs2::msg::ControllerCommand>(CONTROL_ANGULAR_TOPIC, 10);
            statusPub = create_publisher<LedCmd>(LED_COMMAND_TOPIC, 10);

            //determine bag trigger topic for RViz: /<hostname>/autonomy/bag_trigger
            char hostname[HOST_NAME_MAX];
            gethostname(hostname, HOST_NAME_MAX);
            std::string bagTriggerTopic = "/" + std::string(hostname) + "/autonomy/bag_trigger";
            bagTriggerPub = this->create_publisher<std_msgs::msg::Bool>(bagTriggerTopic, 10);

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
            declare_parameter<std::string>("log_file_dir", getEnvVar("HOME") + "/btlogs");
            declare_parameter<std::vector<std::string>>("ext_plugin_list", std::vector<std::string>());
            declare_parameter<std::vector<std::string>>("ext_tree_dirs", std::vector<std::string>());

            // load the params
            try
            {
                RCLCPP_INFO(this->get_logger(), "Getting parameter data");
                // get param values
                enableZMQ = get_parameter("enable_zmq").as_bool();
                treeDirs = get_parameter("ext_tree_dirs").as_string_array();
                pluginPaths = get_parameter("ext_plugin_list").as_string_array();
                fblDirPath = get_parameter("log_file_dir").as_string();
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
            treeDirs.push_back(AUTONOMY_TREE_DIR);
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
            if(!std::filesystem::exists(goal->tree)) {
                RCLCPP_ERROR(log, "Rejecting request to run tree %s because the file does not exist.", goal->tree.c_str());
                return rclcpp_action::GoalResponse::REJECT;
            }

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

                //make a new directory for the FBL log files
                std::filesystem::create_directory(fblDirPath);

                //get time
                time_t now = time(0);
                struct tm tstruct;
                char buf[80];
                tstruct = *localtime(&now);
                strftime(buf, sizeof(buf), "%Y_%m_%d_%X", &tstruct);
                //get file path name using time
                std::string FBLFilePath = fblDirPath + "/BTLog_" + buf + ".fbl";

                // add the loggers to the BT context
                RCLCPP_INFO(log, "DoTask: Loading Monitor");
                PublisherZMQ zmq(tree); // publishes behaviortree data to a groot in real time
                FileLogger fileLogger(tree, FBLFilePath.c_str());
                UwrtLogger uwrtLogger(tree, this->shared_from_this());

                // configure our loggers
                zmq.setEnabled(enableZMQ);
                uwrtLogger.setEnabled(true);

                // set up idle sleep rate
                rclcpp::Rate loop_rate(10ms);

                // indicate to the bagger that autonomy is starting by sending true to the bag trigger
                std_msgs::msg::Bool boolMsg;
                boolMsg.data = true;
                bagTriggerPub->publish(boolMsg);

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
                        tree.haltTree();
                        goal_handle->canceled(result);
                        RCLCPP_INFO(log, "DoTask: Cancelled current action goal");
                        return;
                    }

                    // sleep a bit while we wait
                    loop_rate.sleep();
                }

                // indicate to bagging that the tree is done by sending a false to the bag trigger
                boolMsg.data = false;
                bagTriggerPub->publish(boolMsg);

                //stop the controller. no reason for it to be going
                riptide_msgs2::msg::ControllerCommand disable;
                disable.mode = riptide_msgs2::msg::ControllerCommand::DISABLED;
                linearPub->publish(disable);
                angularPub->publish(disable);

                // other post tree things
                fileLogger.flush();

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

                //publish led command to indicate finish status
                LedCmd ledCmd;
                ledCmd.red   = (tickStatus == BT::NodeStatus::SUCCESS ? 0 : 255);
                ledCmd.green = (tickStatus == BT::NodeStatus::SUCCESS ? 255 : 0);
                ledCmd.blue  = 0;
                ledCmd.mode  = LedCmd::MODE_BREATH;
                ledCmd.target = LedCmd::TARGET_ALL;
                statusPub->publish(ledCmd);

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
            
            // if error, publish led command indicate error status
            LedCmd ledCmd;
            ledCmd.red = 255;
            ledCmd.green = 0;
            ledCmd.blue = 0;
            ledCmd.mode = LedCmd::MODE_FAST_FLASH;
            ledCmd.target = LedCmd::TARGET_ALL;
            statusPub->publish(ledCmd);

            //stop bagging
            std_msgs::msg::Bool boolMsg;
            boolMsg.data = false;
            bagTriggerPub->publish(boolMsg);

            //stop the controller
            riptide_msgs2::msg::ControllerCommand disable;
            disable.mode = riptide_msgs2::msg::ControllerCommand::DISABLED;
            linearPub->publish(disable);
            angularPub->publish(disable);

            // ...then abort
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
        // ros publishers
        rclcpp::Publisher<LedCmd>::SharedPtr statusPub;
        rclcpp::Publisher<riptide_msgs2::msg::ControllerCommand>::SharedPtr 
            linearPub,
            angularPub;
        
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bagTriggerPub;

        // ros action and service servers
        rclcpp_action::Server<ExecuteTree>::SharedPtr actionServer;
        rclcpp::Service<riptide_msgs2::srv::ListTrees>::SharedPtr listTreeServer;

        // logger enablement flags
        bool enableZMQ;

        // execution context thread for the action server
        std::thread executionThread;

        // behavior tree factory context
        std::shared_ptr<BehaviorTreeFactory> factory;

        // full tree file path vector to load
        std::vector<std::string> treeDirs;
        std::vector<std::string> pluginPaths;

        // cout log file location
        std::string fblDirPath;
    };
} // namespace do_task

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    std::string treeDir = AUTONOMY_TREE_DIR;
    RCLCPP_INFO(log, "Using tree directory at %s", treeDir.c_str());

    // create our node context
    auto node = std::make_shared<do_task::BTExecutor>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    // rclcpp::spin(node);
    rclcpp::shutdown();
}