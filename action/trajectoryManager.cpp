#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <riptide_autonomy/TrajectoryAction.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <fcl/fcl.h>
#include <ompl-1.5/ompl/base/State.h>

/**
 * Source file for the OSU-UWRT trajectory manager action server.
 */

const std::string 
    ODOMETRY_TOPIC = "/puddles/odometry",
    OCTOMAP_TOPIC = "/octomap/octomap_binary";

const double
    OCTOMAP_RESOLUTION = 0.01;

class TrajectoryAction {
    protected:
    //actionlib stuff
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<riptide_autonomy::TrajectoryAction> actionServer;
    std::string actionName;

    riptide_autonomy::TrajectoryActionFeedback feedback;
    riptide_autonomy::TrajectoryActionResult result;

    //publishers and subscribers
    ros::Subscriber octoSub;

    //data from subs
    octomap::OcTree *environment;
    bool environmentIsPublished = false;

    public:
    TrajectoryAction(std::string name) :
        actionServer(nh, name, boost::bind(&TrajectoryAction::executeCallback, this, _1), false),
        actionName(name)
    {
        actionServer.start();
    }

    ~TrajectoryAction(void) {
    }

    void executeCallback(const riptide_autonomy::TrajectoryGoalConstPtr &goal) {
        while(ros::ok() && !atGoal()) {
            
            ros::spinOnce();
        }
        
        actionServer.setSucceeded(result.result);
    }

    private:

    /**
     * Returns true if the robot is at the goal, false otherwise.
     * TODO: implement
     */
    bool atGoal() {
        return false;
    }

    /**
     * Returns true if the ompl path is valid, false otherwise.
     */
    bool pathValid(const ompl::base::State *state) {

        return false;
    }

    /**
     * When subscriber gets data
     */
    void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg) {
        octomap::AbstractOcTree *abstract = octomap_msgs::binaryMsgToMap(*msg.get());
        if(abstract) {
            environment = dynamic_cast<octomap::OcTree*>(abstract);
            environmentIsPublished = true;
        }
    }

};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "Trajectory");
    TrajectoryAction trajectory("Trajectory");
    ROS_INFO("Trajectory Manager Started.");
    ros::spin();

    return 0;
}