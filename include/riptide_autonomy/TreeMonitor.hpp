#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/abstract_logger.h>

/**
 * @brief Read-only live view of the trees doTask runs, for notgroot. One ZeroMQ REP socket serves any number of
 * clients; nothing they send changes the tree. The last run stays until the next one starts, so a client that
 * connects late or after the end still gets all of it. The protocol is in notgroot's docs/contract.md.
 */
class TreeMonitor {
    public:
    /**
     * @brief One run: a .btlog's header and transitions, plus the tree while it is alive (for blackboards).
     */
    struct Run {
        std::string id, mission, logPath;
        int64_t startUsec = 0;
        std::string xml;

        std::mutex mutex;
        std::vector<uint8_t> transitions;
        bool ended = false;
        std::string result = "RUNNING";

        std::mutex treeMutex;
        const BT::Tree *tree = nullptr;

        std::once_flag zipped;
        std::string zippedXml;
    };

    /**
     * @brief Records one tree's transitions into its Run. Destroy it before the tree.
     */
    class Tap : public BT::StatusChangeLogger {
        public:
        Tap(const BT::Tree& tree, std::shared_ptr<Run> run);
        ~Tap() override;

        void callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prevStatus,
                      BT::NodeStatus status) override;
        void flush() override { }

        /**
         * @brief Ends the run with result (SUCCESS, FAILURE, CANCELED...). A tap destroyed without it ends as ERROR.
         */
        void end(const std::string& result);

        private:
        std::shared_ptr<Run> run;
    };

    TreeMonitor() = default;
    ~TreeMonitor();

    TreeMonitor(const TreeMonitor&) = delete;
    TreeMonitor& operator=(const TreeMonitor&) = delete;

    /**
     * @brief Binds the port on every interface and starts serving. Returns an empty string, or why it could not.
     */
    std::string start(int port);

    /**
     * @brief Starts a new run, replacing the last one. Call it right after the FileLogger2 for logPath is made and
     * flushed: the run takes that log's header, so both hold the same tree and start time.
     */
    std::unique_ptr<Tap> beginRun(const BT::Tree& tree, const std::string& mission, const std::string& logPath);

    private:
    void serve();
    std::vector<std::string> answer(const std::vector<std::string>& request);

    void *context = nullptr;
    void *socket = nullptr;
    std::thread server;
    std::atomic<bool> running{false};

    std::mutex runMutex;
    std::shared_ptr<Run> current;
};
