#include "autonomy.h"

using namespace BT;

/**
 * @brief Velocity state.
 * 
 * This state drives the robot at a constant velocity while fixed at its
 * current orientation (its orientation when the state starts)
 */
class VelocityState : public UWRTSyncActionNode {
    public:

    VelocityState(const std::string& name, const NodeConfiguration& config)
     : UWRTSyncActionNode(name, config) { }

    /**
     * @brief Initializes the node. 
     * 
     * @param node The ROS node belonging to the current process.
     */
    void init(rclcpp::Node::SharedPtr node) {
        rosnode = node;

        //any publishers or subscribers should be initialized here...
    }

    /**
     * @brief Declares ports needed by this state.
     * 
     * @return PortsList Needed ports.
     */
    static PortsList providedPorts() {
        return {
            InputPort<std::string>("time"),
            InputPort<std::string>("x_velocity"),
            InputPort<std::string>("y_velocity"),
            InputPort<std::string>("z_velocity")
        };
    }

    /**
     * @brief Executes the node.
     * This method will be called once by the tree and can block for as long
     * as it needs for the action to be completed. When execution completes,
     * this method must return either SUCCESS or FAILURE; it CANNOT return 
     * IDLE or RUNNING.
     * 
     * @return NodeStatus The result of the execution; SUCCESS or FAILURE.
     */
    NodeStatus tick() override {
        while(rclcpp::ok()) {
            if(!hasEntered) {
                //initial calculations for movement
                startTime = rosnode->now().seconds();
                hasEntered = true;

                //figure out velocities to drive the robot at
                Optional<std::string> 
                    xVelStr = getInput<std::string>("x_velocity"),
                    yVelStr = getInput<std::string>("y_velocity"),
                    zVelStr = getInput<std::string>("z_velocity");
                
                //grab velocities from behaviotree parameters
                double 
                    xVel = std::stod(xVelStr.value()),
                    yVel = std::stod(yVelStr.value()),
                    zVel = std::stod(zVelStr.value());
                
                //assemble Vector3 that will be published while robot is moving
                velocities.x = xVel;
                velocities.y = yVel;
                velocities.z = zVel;

                //wait for localization data so that we can grab the orientation to lock to
                RCLCPP_INFO(log, "Waiting for odometry/filtered...");
                while(!odomDataExists) {
                    rclcpp::spin_some(rosnode);
                }

                //grab orientation to use while moving
                orientation.w = currentPose.orientation.w;
                orientation.x = currentPose.orientation.x;
                orientation.y = currentPose.orientation.y;
                orientation.z = currentPose.orientation.z;
            }
            
            //publish velocity data
            velocityPub->publish(velocities);
            orientationPub->publish(orientation);

            //get time data from port
            Optional<std::string> timeStr = getInput<std::string>("time");
            int goalTime = std::stoi(timeStr.value());
            int elapsedTime = rosnode->now().seconds() - startTime;

            if(elapsedTime >= goalTime) { //the command be done
                geometry_msgs::msg::Vector3 zeroVelocity;
                velocityPub->publish(zeroVelocity);
                return NodeStatus::SUCCESS;
            }
        }

        return NodeStatus::FAILURE;
    }

    private:
    /**
     * @brief Called when the odometry subscription receives a message.
     * 
     * @param msg The received message.
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        this->currentPose = msg->pose.pose;
        odomDataExists = true;
    }

    rclcpp::Node::SharedPtr rosnode;

    //publishers 
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr velocityPub;
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr orientationPub;

    //subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;

    //goal information
    geometry_msgs::msg::Vector3 velocities;
    geometry_msgs::msg::Quaternion orientation;

    //other information
    geometry_msgs::msg::Pose currentPose;

    bool
        hasEntered,
        odomDataExists;

    int startTime;
};