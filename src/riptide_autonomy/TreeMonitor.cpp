#include "riptide_autonomy/TreeMonitor.hpp"

#include <behaviortree_cpp/xml_parsing.h>

#include <chrono>
#include <cstring>
#include <fstream>

#include <zlib.h>
#include <zmq.h>

namespace {
    // a TRANS reply holds at most this many records, the client asks again from where it got to
    const size_t MAX_RECORDS = 65536;
    const size_t RECORD_SIZE = 9;

    int64_t toUsec(BT::Duration duration) {
        return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
    }

    // the header FileLogger2 wrote: magic, protocol, int32 xml size, xml, int64 start time
    bool readLogHeader(const std::string& path, std::string& xml, int64_t& startUsec) {
        std::ifstream file(path, std::ios::binary);
        const std::string magic = "BTCPP4-FileLogger2";
        std::string head(magic.size() + 1, '\0');
        if(!file.read(head.data(), head.size()) || head.compare(0, magic.size(), magic) != 0) {
            return false;
        }

        int32_t size = 0;
        if(!file.read(reinterpret_cast<char *>(&size), 4) || size <= 0) {
            return false;
        }

        xml.resize(size);
        return file.read(xml.data(), size) && file.read(reinterpret_cast<char *>(&startUsec), 8);
    }

    std::string zip(const std::string& text) {
        uLongf size = compressBound(text.size());
        std::string out(size, '\0');
        if(compress2(reinterpret_cast<Bytef *>(out.data()), &size, reinterpret_cast<const Bytef *>(text.data()),
                     text.size(), Z_DEFAULT_COMPRESSION) != Z_OK) {
            return "";
        }

        out.resize(size);
        return out;
    }

    bool parseIndex(const std::string& text, size_t& index) {
        if(text.empty() || text.find_first_not_of("0123456789") != std::string::npos) {
            return false;
        }

        index = std::stoull(text);
        return true;
    }
}


TreeMonitor::Tap::Tap(const BT::Tree& tree, std::shared_ptr<Run> run) : StatusChangeLogger(), run(run) {
    // same records as FileLogger2, idle transitions included
    enableTransitionToIdle(true);
    {
        std::lock_guard<std::mutex> lock(run->treeMutex);
        run->tree = &tree;
    }

    subscribeToTreeChanges(tree.rootNode());
}


TreeMonitor::Tap::~Tap() {
    unsubscribeFromTreeChanges();
    end("ERROR");

    // the tree goes after this, so its blackboards can't be read anymore
    std::lock_guard<std::mutex> lock(run->treeMutex);
    run->tree = nullptr;
}


void TreeMonitor::Tap::callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prevStatus,
                                BT::NodeStatus status) {
    (void) prevStatus;

    const uint64_t offset = toUsec(timestamp) - run->startUsec;
    const uint16_t uid = node.UID();
    const uint8_t code = static_cast<uint8_t>(status);

    uint8_t record[RECORD_SIZE];
    std::memcpy(record, &offset, 6);
    std::memcpy(record + 6, &uid, 2);
    record[8] = code;

    std::lock_guard<std::mutex> lock(run->mutex);
    run->transitions.insert(run->transitions.end(), record, record + RECORD_SIZE);
}


void TreeMonitor::Tap::end(const std::string& result) {
    std::lock_guard<std::mutex> lock(run->mutex);
    if(!run->ended) {
        run->ended = true;
        run->result = result;
    }
}


TreeMonitor::~TreeMonitor() {
    running = false;
    if(server.joinable()) {
        server.join();
    }

    if(socket) {
        zmq_close(socket);
    }

    if(context) {
        zmq_ctx_term(context);
    }
}


std::string TreeMonitor::start(int port) {
    context = zmq_ctx_new();
    socket = zmq_socket(context, ZMQ_REP);

    const int linger = 0;
    zmq_setsockopt(socket, ZMQ_LINGER, &linger, sizeof(linger));

    const std::string address = "tcp://*:" + std::to_string(port);
    if(zmq_bind(socket, address.c_str()) != 0) {
        const std::string error = zmq_strerror(zmq_errno());
        zmq_close(socket);
        socket = nullptr;
        return error;
    }

    running = true;
    server = std::thread(&TreeMonitor::serve, this);
    return "";
}


std::unique_ptr<TreeMonitor::Tap> TreeMonitor::beginRun(const BT::Tree& tree, const std::string& mission,
                                                        const std::string& logPath) {
    auto run = std::make_shared<Run>();
    run->mission = mission;
    run->logPath = logPath;

    // no log to copy: build the same header ourselves
    if(!readLogHeader(logPath, run->xml, run->startUsec)) {
        run->xml = BT::WriteTreeToXML(tree, true, true);
        run->startUsec = toUsec(std::chrono::system_clock::now().time_since_epoch());
    }

    run->id = std::to_string(run->startUsec);
    auto tap = std::make_unique<Tap>(tree, run);

    std::lock_guard<std::mutex> lock(runMutex);
    current = run;
    return tap;
}


void TreeMonitor::serve() {
    while(running) {
        zmq_pollitem_t item = {socket, 0, ZMQ_POLLIN, 0};
        if(zmq_poll(&item, 1, 200) <= 0 || !(item.revents & ZMQ_POLLIN)) {
            continue;
        }

        // every frame of the request
        std::vector<std::string> request;
        int more = 1;
        while(more) {
            zmq_msg_t msg;
            zmq_msg_init(&msg);
            if(zmq_msg_recv(&msg, socket, 0) < 0) {
                zmq_msg_close(&msg);
                break;
            }

            request.emplace_back(static_cast<char *>(zmq_msg_data(&msg)), zmq_msg_size(&msg));
            more = zmq_msg_more(&msg);
            zmq_msg_close(&msg);
        }

        std::vector<std::string> reply;
        try {
            reply = answer(request);
        } catch(const std::exception& e) {
            reply = {"ERR", e.what()};
        }

        for(size_t i = 0; i < reply.size(); i++) {
            zmq_send(socket, reply[i].data(), reply[i].size(), i + 1 < reply.size() ? ZMQ_SNDMORE : 0);
        }
    }
}


std::vector<std::string> TreeMonitor::answer(const std::vector<std::string>& request) {
    if(request.empty()) {
        return {"ERR", "empty request"};
    }

    std::shared_ptr<Run> run;
    {
        std::lock_guard<std::mutex> lock(runMutex);
        run = current;
    }

    const std::string& command = request[0];
    if(command == "RUN") {
        if(!run) {
            return {"NONE"};
        }

        nlohmann::json meta = {
            {"protocol", 1}, {"run", run->id}, {"mission", run->mission}, {"log", run->logPath},
            {"start_usec", run->startUsec}, {"xml_bytes", run->xml.size()}};
        std::lock_guard<std::mutex> lock(run->mutex);
        meta["transitions"] = run->transitions.size() / RECORD_SIZE;
        meta["ended"] = run->ended;
        meta["result"] = run->result;
        return {"OK", meta.dump()};
    }

    if(command != "TREE" && command != "TRANS" && command != "BB") {
        return {"ERR", "unknown request " + command};
    }

    // the rest name a run, which must still be the current one
    if(request.size() < 2 || !run || request[1] != run->id) {
        return {"ERR", "unknown run"};
    }

    if(command == "TREE") {
        std::call_once(run->zipped, [&run]() { run->zippedXml = zip(run->xml); });
        return {"OK", run->zippedXml};
    }

    if(command == "TRANS") {
        size_t from = 0;
        if(request.size() < 3 || !parseIndex(request[2], from)) {
            return {"ERR", "TRANS needs a start index"};
        }

        std::lock_guard<std::mutex> lock(run->mutex);
        const size_t count = run->transitions.size() / RECORD_SIZE;
        from = std::min(from, count);
        const size_t to = std::min(count, from + MAX_RECORDS);

        nlohmann::json meta = {
            {"from", from}, {"to", to}, {"count", count}, {"ended", run->ended}, {"result", run->result}};
        const auto first = run->transitions.begin() + from * RECORD_SIZE;
        return {"OK", meta.dump(), std::string(first, first + (to - from) * RECORD_SIZE)};
    }

    // BB: one json object per instance path asked for, missing ones left out
    std::lock_guard<std::mutex> lock(run->treeMutex);
    if(!run->tree) {
        return {"ERR", "run ended"};
    }

    nlohmann::json blackboards = nlohmann::json::object();
    for(size_t i = 2; i < request.size(); i++) {
        for(const auto& subtree : run->tree->subtrees) {
            if(subtree->instance_name == request[i]) {
                const auto values = BT::ExportBlackboardToJSON(*subtree->blackboard);
                blackboards[request[i]] = values.is_null() ? nlohmann::json::object() : values;
                break;
            }
        }
    }

    return {"OK", blackboards.dump(-1, ' ', false, nlohmann::json::error_handler_t::replace)};
}
