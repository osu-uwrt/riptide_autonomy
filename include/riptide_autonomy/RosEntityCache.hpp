#pragma once

#include <functional>
#include <map>
#include <mutex>
#include <sstream>
#include <typeinfo>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

/**
 * @brief Keeps a callback registered with a SharedSubscription; destroying it unregisters the callback.
 * Declare it after everything its callback uses, so it goes first when the node is destroyed.
 */
class RosListener {
    public:
    explicit RosListener(std::function<void()> stop) : stop(stop) { }
    ~RosListener() {
        stop();
    }

    RosListener(const RosListener&) = delete;
    RosListener& operator=(const RosListener&) = delete;

    private:
    std::function<void()> stop;
};

using RosListenerPtr = std::unique_ptr<RosListener>;

/**
 * @brief One subscription that hands each message to every registered listener, and keeps the latest one.
 * Get it from RosEntityCache so every node on the same topic, type and QoS shares it.
 * Callbacks run with the listener list locked: a callback must not listen, and nothing may listen while
 * holding a lock its own callback takes.
 */
template<typename MsgT>
class SharedSubscription : public std::enable_shared_from_this<SharedSubscription<MsgT>> {
    public:
    using Callback = std::function<void(const typename MsgT::SharedPtr)>;

    /**
     * @brief Creates the ROS subscription. Called once, by RosEntityCache.
     */
    void start(rclcpp::Node::SharedPtr node, const std::string& topic, const rclcpp::QoS& qos) {
        std::weak_ptr<SharedSubscription> weak = this->shared_from_this();
        sub = node->create_subscription<MsgT>(topic, qos, [weak](const typename MsgT::SharedPtr msg) {
            if(auto self = weak.lock()) {
                self->dispatch(msg);
            }
        });
    }

    /**
     * @brief Calls callback with every message from now on, until the returned listener is destroyed.
     */
    RosListenerPtr listen(Callback callback) {
        std::lock_guard<std::mutex> lock(mutex);
        const uint64_t id = nextId++;
        listeners[id] = callback;

        auto self = this->shared_from_this();
        return std::make_unique<RosListener>([self, id]() {
            std::lock_guard<std::mutex> lock(self->mutex);
            self->listeners.erase(id);
        });
    }

    /**
     * @brief The latest message received, or nullptr if there was none yet.
     */
    typename MsgT::SharedPtr latest() {
        std::lock_guard<std::mutex> lock(mutex);
        return latestMsg;
    }

    size_t listenerCount() {
        std::lock_guard<std::mutex> lock(mutex);
        return listeners.size();
    }

    std::string topicName() const {
        return sub->get_topic_name();
    }

    private:
    void dispatch(const typename MsgT::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex);
        latestMsg = msg;
        for(auto& [id, callback] : listeners) {
            callback(msg);
        }
    }

    std::mutex mutex;
    std::map<uint64_t, Callback> listeners;
    uint64_t nextId = 0;
    typename MsgT::SharedPtr latestMsg;
    typename rclcpp::Subscription<MsgT>::SharedPtr sub;
};

/**
 * @brief Per-process cache of the ROS entities BT nodes use: one subscription, publisher, service client,
 * action client or parameter client per name, type and QoS, shared by every node that asks for it. An entry
 * lives as long as some node holds it, so a tree's entities go away with the tree.
 */
class RosEntityCache {
    public:
    /**
     * @brief Live entities by kind ("pub", "sub", "client", "action", "params").
     */
    static std::map<std::string, size_t> liveCounts();

    /**
     * @brief Live entities of every kind.
     */
    static size_t liveTotal();

    template<typename MsgT>
    static std::shared_ptr<SharedSubscription<MsgT>> subscription(rclcpp::Node::SharedPtr node, const std::string& topic, const rclcpp::QoS& qos) {
        return getOrCreate<SharedSubscription<MsgT>>("sub", node, resolve(node, topic), typeid(MsgT).name(), qosKey(qos), [&]() {
            auto shared = std::make_shared<SharedSubscription<MsgT>>();
            shared->start(node, topic, qos);
            return shared;
        });
    }

    template<typename MsgT>
    static typename rclcpp::Publisher<MsgT>::SharedPtr publisher(rclcpp::Node::SharedPtr node, const std::string& topic, const rclcpp::QoS& qos) {
        return getOrCreate<rclcpp::Publisher<MsgT>>("pub", node, resolve(node, topic), typeid(MsgT).name(), qosKey(qos), [&]() {
            return node->create_publisher<MsgT>(topic, qos);
        });
    }

    template<typename SrvT>
    static typename rclcpp::Client<SrvT>::SharedPtr client(rclcpp::Node::SharedPtr node, const std::string& service) {
        return getOrCreate<rclcpp::Client<SrvT>>("client", node, resolve(node, service, true), typeid(SrvT).name(), "", [&]() {
            return node->create_client<SrvT>(service);
        });
    }

    template<typename ActionT>
    static typename rclcpp_action::Client<ActionT>::SharedPtr actionClient(rclcpp::Node::SharedPtr node, const std::string& action) {
        return getOrCreate<rclcpp_action::Client<ActionT>>("action", node, resolve(node, action, true), typeid(ActionT).name(), "", [&]() {
            return rclcpp_action::create_client<ActionT>(node, action);
        });
    }

    static rclcpp::AsyncParametersClient::SharedPtr parametersClient(rclcpp::Node::SharedPtr node, const std::string& remoteNode);

    private:
    template<typename T>
    static std::shared_ptr<T> getOrCreate(
        const std::string& kind,
        rclcpp::Node::SharedPtr node,
        const std::string& name,
        const std::string& type,
        const std::string& qos,
        std::function<std::shared_ptr<T>()> create)
    {
        std::ostringstream key;
        key << kind << '|' << node.get() << '|' << name << '|' << type << '|' << qos;

        std::lock_guard<std::mutex> lock(mutex);
        auto& entry = entries[key.str()];
        if(auto existing = std::static_pointer_cast<T>(entry.lock())) {
            return existing;
        }

        auto created = create();
        entry = created;
        prune();
        return created;
    }

    static void prune();

    static std::string resolve(rclcpp::Node::SharedPtr node, const std::string& name, bool isService = false);
    static std::string qosKey(const rclcpp::QoS& qos);

    static std::mutex mutex;
    static std::map<std::string, std::weak_ptr<void>> entries;
};
