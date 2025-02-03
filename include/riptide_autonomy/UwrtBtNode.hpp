#pragma once

#include "riptide_autonomy/autonomy_base.hpp"

class ROSEnabledNode {
    public:

    static void staticInit(rclcpp::Node::SharedPtr node);
    static void staticDeinit();

    void init(rclcpp::Node::SharedPtr node);
    const rclcpp::Node::SharedPtr rosNode() const;

    protected:
    virtual void rosInit() = 0;
    static std::shared_ptr<tf2_ros::Buffer> tfBuffer;

    private:
    rclcpp::Node::SharedPtr rosnode;
    static std::shared_ptr<tf2_ros::TransformListener> tfListener;
};


template<typename NodeType>

class UwrtBtNode : public NodeType, public ROSEnabledNode
{
    public:
    UwrtBtNode(const std::string& name, const BT::NodeConfig& config)
     : NodeType(name, config) { }

    protected:

    /**
     * @brief Get a thing from a BT blackboard.
     * 
     * @tparam T The type of the pointer to grab.
     * @param n The UwrtBtNode to get the bb value from
     * @param key The name of the value to grab.
     * @param value The variable to be populated with the desired blackboard entry.
     * @return true If the operation succeeds
     * @return false If the operation fails
     */
    template<typename T>
    bool getFromBlackboard(rclcpp::Node::SharedPtr rosnode, BT::Blackboard::Ptr bb, const std::string& key, T& value) {
        try {
            if(bb->get<T>(key, value)) {
                return true;
            }
        } catch (std::runtime_error& ex) {
            RCLCPP_ERROR(this->rosNode()->get_logger(), "Error getting blackboard value named \"%s\": %s", key.c_str(), ex.what());
        }

        return false;
    }

    /**
     * @brief Get a thing from a BT blackboard.
     * 
     * @tparam T The type of the pointer to grab.
     * @param n The UwrtBtNode to get the bb value from
     * @param key The name of the value to grab.
     * @param value The variable to be populated with the desired blackboard entry.
     * @return true If the operation succeeds
     * @return false If the operation fails
     */
    template<typename T>
    bool getFromBlackboard(const std::string& key, T& value) {
        if(!this->config().blackboard) {
            RCLCPP_ERROR(this->rosNode()->get_logger(), "Cannot get from blackboard! The passed TreeNode does not have one!");
            return false;
        }

        return this->template getFromBlackboard<T>(this->rosNode(), this->config().blackboard, key, value);
    }


    template<typename T>
    void postOutput(const std::string& key, T value) {
        std::ostringstream stream;
        stream << value;

        NodeType::setOutput(key, stream.str());
    }

    /**
     * @brief Attempts to get a port input from a TreeNode.
     * 
     * @tparam T The type of the port value to get.
     * @param n The node to get the port value of.
     * @param key The key of the port.
     * @param defaultValue The value to return if the key does not have a value
     * @param warnIfUndefined True if a warning should be printed if the key doesn't exist
     * @return T The value of the port or the default if there is none.
     */
    template<typename T> 
    T tryGetInput(const std::string& key, const T defaultValue, bool warnIfUndefined) {
        auto op = this->template getInput<std::string>(key);
        if(op.has_value()) {
            return BT::convertFromString<T>(op.value());
        } else if(warnIfUndefined) {
            RCLCPP_WARN(this->rosNode()->get_logger(), "Node %s does not have a value for required port with name %s!", this->name().c_str(), key.c_str());
        }

        return defaultValue;
    }

    /**
     * @brief Gets an input from a required port. A warning will be printed if the value does not exist
     * 
     * @tparam T The type of the port value.
     * @param n The TreeNode to get the value from
     * @param key The name of the value to get
     * @param defaultValue The value to return if the key does not have a value.
     * @return T The value of the port, or the default value if there is no such value.
     */
    template<typename T>
    T tryGetRequiredInput(const std::string& key, const T defaultValue) {
        return this->template tryGetInput<T>(key, defaultValue, true);
    }

    /**
     * @brief Attempts to get an optional value from a port.
     * 
     * @tparam T The type of the value to get.
     * @param n The TreeNode to get the port value from
     * @param key The name of the value to get.
     * @param defaultValue the value fto retuurn if the key does not have a value.
     * @return T The value of the port, or the default value if there is no such value.
     */
    template<typename T>
    T tryGetOptionalInput(const std::string& key, const T defaultValue) {
        return this->template tryGetInput<T>(key, defaultValue, false);
    }


    /**
     * @brief Attempts to get an entry from a BT blackboard, and returns a specified default value if the operation fails.
     * 
     * @tparam T The type of value to get.
     * @param blackboard The blackboard to use.
     * @param key The key to grab from the blackboard.
     * @param defaultValue The value to return if the operation fails
     * @return T The value of the blackboard entry, or the default value if it cannot be retrieved.
     */
    template<typename T>
    T getFromBlackboardWithDefault(BT::Blackboard::Ptr blackboard, const std::string& key, const T defaultValue) {
        T retval = defaultValue;
        this->getFromBlackboard(blackboard, key, retval);
        return retval;
    }

    /**
     * @brief Attempts to get an entry from a BT blackboard, and returns a specified default value if the operation fails.
     * 
     * @tparam T The type of value to get.
     * @param n The tree node whose blackboard to use.
     * @param key The key to grab from the blackboard.
     * @param defaultValue The value to return if the operation fails.
     * @return T The value of the blackboard entry, or the default value if it cannot be retrieved.
     */
    template<typename T>
    T getFromBlackboardWithDefault(const std::string& key, T& defaultValue) {
        T retval = defaultValue;
        this->getFromBlackboard(key, retval);
        return retval;
    }

    /**
     * @brief Inserts blackboard entries, where applicable, into the given string.
     *
     * @param str The string to populate from the blackboard.
     * @param treeNode The behaviortree node to grab the blackboard values from.
     * @return std::string The string with the blackboard entries inserted
     */
    std::string formatStringWithBlackboard(const std::string& str)
    {
        std::string result = "";
        int pos = 0;
        while(str.find_first_of('{', pos) != std::string::npos) {
            int lbpos = str.find_first_of("{", pos);
            result += str.substr(pos, lbpos - pos); //add everything from pos up until the "{" to result

            if(str.find_first_of("}", lbpos) != std::string::npos) {
                int rbpos = str.find_first_of("}", lbpos);

                std::string 
                    tokenWithBrackets = str.substr(lbpos, rbpos - lbpos + 1),
                    nameOfEntry = tokenWithBrackets.substr(1, tokenWithBrackets.length() - 2),
                    valueOfEntry;

                //get the value of the entry
                if(this->template getFromBlackboard<std::string>(nameOfEntry, valueOfEntry)) {
                    //if nameOfEntry exists, valueOfEntry was populated by the call above
                    result += valueOfEntry;
                } else {
                    result += tokenWithBrackets; //put whole token in because it didn't lead anywhere
                }

                pos = rbpos + 1; //set position to after '}'
            } else {
                pos = lbpos + 1; //set position to after '{ (there is no '}')
            }
        }

        result += str.substr(pos);
        return result;
    }
};


typedef UwrtBtNode<BT::StatefulActionNode> UWRTActionNode;
typedef UwrtBtNode<BT::ConditionNode> UWRTConditionNode;
typedef UwrtBtNode<BT::DecoratorNode>UWRTDecoratorNode;

// /**
//  * @brief UWRT superclass for BT action nodes
//  */
// class UWRTActionNode : virtual public BT::StatefulActionNode, public UwrtBtNode {
//     public:
//     UWRTActionNode(const std::string& name, const BT::NodeConfiguration& config)
//      : StatefulActionNode(name, config) { };
// };

// /**
//  * @brief UWRT superclass for integrating ConditionNodes with ROS.
//  * Similar to UWRTSyncActionNode, this class inherits both the BT 
//  * ConditionNode and UwrtBtNode.
//  */
// class UWRTConditionNode : virtual public BT::ConditionNode, public UwrtBtNode {
//     public:
//     UWRTConditionNode(const std::string& name, const BT::NodeConfiguration& config)
//      : ConditionNode(name, config) { };
// };

// /**
//  * @brief UWRT superclass for integrating DecoratorNodes with ROS.
//  * Operates exactly the same as UWRTConditionNode and UWRTActionNode.
//  */
// class UWRTDecoratorNode : virtual public BT::DecoratorNode, public UwrtBtNode {
//     public:
//     UWRTDecoratorNode(const std::string& name, const BT::NodeConfiguration& config)
//      : DecoratorNode(name, config) { };
// };
