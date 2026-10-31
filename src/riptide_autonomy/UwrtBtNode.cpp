#include "riptide_autonomy/UwrtBtNode.hpp"

std::shared_ptr<tf2_ros::Buffer> UwrtBtNode::tfBuffer = nullptr;
std::shared_ptr<tf2_ros::TransformListener> UwrtBtNode::tfListener = nullptr;

std::mutex RosEntityCache::mutex;
std::map<std::string, std::weak_ptr<void>> RosEntityCache::entries;


std::map<std::string, size_t> RosEntityCache::liveCounts() {
    std::lock_guard<std::mutex> lock(mutex);
    std::map<std::string, size_t> counts;
    for(const auto& [key, entry] : entries) {
        if(!entry.expired()) {
            counts[key.substr(0, key.find('|'))]++;
        }
    }

    return counts;
}


size_t RosEntityCache::liveTotal() {
    size_t total = 0;
    for(const auto& [kind, count] : liveCounts()) {
        total += count;
    }

    return total;
}


rclcpp::AsyncParametersClient::SharedPtr RosEntityCache::parametersClient(rclcpp::Node::SharedPtr node, const std::string& remoteNode) {
    return getOrCreate<rclcpp::AsyncParametersClient>("params", node, remoteNode, "", "", [&]() {
        return std::make_shared<rclcpp::AsyncParametersClient>(node, remoteNode);
    });
}


void RosEntityCache::prune() {
    for(auto it = entries.begin(); it != entries.end();) {
        it = it->second.expired() ? entries.erase(it) : std::next(it);
    }
}


std::string RosEntityCache::resolve(rclcpp::Node::SharedPtr node, const std::string& name, bool isService) {
    // the name the entity really gets, so "odometry/filtered" and "/talos/odometry/filtered" share
    char *resolved = nullptr;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    if(rcl_node_resolve_name(node->get_node_base_interface()->get_rcl_node_handle(), name.c_str(), allocator, isService, false, &resolved) != RCL_RET_OK) {
        rcl_reset_error();
        return name;
    }

    std::string result(resolved);
    allocator.deallocate(resolved, allocator.state);
    return result;
}


std::string RosEntityCache::qosKey(const rclcpp::QoS& qos) {
    const rmw_qos_profile_t& p = qos.get_rmw_qos_profile();
    std::ostringstream key;
    key << p.history << ',' << p.depth << ',' << p.reliability << ',' << p.durability << ',' << p.liveliness << ','
        << p.deadline.sec << '.' << p.deadline.nsec << ',' << p.lifespan.sec << '.' << p.lifespan.nsec << ','
        << p.liveliness_lease_duration.sec << '.' << p.liveliness_lease_duration.nsec << ',' << p.avoid_ros_namespace_conventions;
    return key.str();
}

void UwrtBtNode::staticInit(rclcpp::Node::SharedPtr node) {
    if(!tfBuffer)
    {
        tfBuffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    }

    if(!tfListener)
    {
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
    }
}


void UwrtBtNode::staticDeinit() {
    if(tfBuffer)
    {
        tfBuffer.reset();
    }

    if(tfListener)
    {
        tfListener.reset();
    }
}


void UwrtBtNode::init(rclcpp::Node::SharedPtr node) {
    this->rosnode = node;
    rosInit();
}


bool UwrtBtNode::lookupTransform(
    const std::string& fromFrame,
    const std::string& toFrame,
    geometry_msgs::msg::TransformStamped& transform,
    bool useCurrentTime)
{
    try {
        tf2::TimePoint tp = (useCurrentTime ? tf2_ros::fromRclcpp(this->rosnode->get_clock()->now()) : tf2::TimePointZero);
        transform = tfBuffer->lookupTransform(toFrame, fromFrame, tp);
        return true;
    } catch(tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(rosnode->get_logger(), *rosnode->get_clock(), 1000, "Failed to look up transform from %s to %s (%s)", fromFrame.c_str(), toFrame.c_str(), ex.what());
    }
    
    return false;
}
