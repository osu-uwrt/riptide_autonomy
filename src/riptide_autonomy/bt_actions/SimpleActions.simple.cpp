#include "riptide_autonomy/simple_nodes.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>


/**
 * @brief Prints a string to the rclcpp info console.
 * 
 * @param n The BehaviorTree node.
 * @return BT::NodeStatus return status.
 */
BT::NodeStatus printInfo(UwrtBtNode& n) {
    std::string message = tryGetRequiredInput<std::string>(&n, "message", "");
    RCLCPP_INFO(n.rosNode()->get_logger(), "%s", formatStringWithBlackboard(message, &n).c_str()); 
    return BT::NodeStatus::SUCCESS; 
}

/**
 * @brief Prints a string to the rclcpp error console.
 * 
 * @param n The BehaviorTree node.
 * @return BT::NodeStatus return status.
 */
BT::NodeStatus printError(UwrtBtNode& n) {
    std::string message = tryGetRequiredInput<std::string>(&n, "message", "");
    RCLCPP_ERROR(n.rosNode()->get_logger(), "%s", formatStringWithBlackboard(message, &n).c_str());
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief Calculates the distance between two 3d points and returns it to the appropriate port.
 * 
 * @param n The BehaviorTree node.
 * @return BT::NodeStatus return status
 */
BT::NodeStatus calculateDistance(UwrtBtNode& n) {
    geometry_msgs::msg::Vector3 p1, p2;
    p1.x = tryGetRequiredInput<double>(&n, "x1", 0);
    p1.y = tryGetRequiredInput<double>(&n, "y1", 0);
    p1.z = tryGetRequiredInput<double>(&n, "z1", 0);
    p2.x = tryGetRequiredInput<double>(&n, "x2", 0);
    p2.y = tryGetRequiredInput<double>(&n, "y2", 0);
    p2.z = tryGetRequiredInput<double>(&n, "z2", 0);

    postOutput<double>(&n, "dist", distance(p1, p2));
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief Adds, subtracts, mutliplies, or divides two numbers and returns the result to the appropriate port.
 * 
 * @param n The BehaviorTree node.
 * @return BT::NodeStatus return status
 */
BT::NodeStatus doMath(UwrtBtNode& n) {
    double
        a = tryGetRequiredInput<double>(&n, "a", 0),
        b = tryGetRequiredInput<double>(&n, "b", 0),
        output = 0;
    
    std::string op = tryGetRequiredInput<std::string>(&n, "operator", "+");

    if(op == "+") {
        output = a + b;
    } else if(op == "-") {
        output = a - b;
    } else if(op == "*") {
        output = a * b;
    } else if(op == "/") {
        output = a / b;
    }

    postOutput<double>(&n, "out", output);
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief Formats a string for BT
 * @param n Tree node
 * @return execution status
 */
BT::NodeStatus format(UwrtBtNode &n) {
    std::string formatStr = tryGetRequiredInput<std::string>(&n, "format", "");
    std::string out = formatStringWithBlackboard(formatStr, &n);
    postOutput<std::string>(&n, "out", out);
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief Get the Heading To Point
 * 
 * @param n The BehaviorTree node.
 * @return BT::NodeStatus return status
 */
BT::NodeStatus getHeadingToPoint(UwrtBtNode& n) {
    double
        currX = tryGetRequiredInput<double>(&n, "currX", 0),
        currY = tryGetRequiredInput<double>(&n, "currY", 0),
        targX = tryGetRequiredInput<double>(&n, "targX", 0),
        targY = tryGetRequiredInput<double>(&n, "targY", 0);

    double
        dx = targX - currX,
        dy = targY - currY;

    double heading = atan2(dy, dx);

    postOutput<double>(&n, "heading", heading);
    return BT::NodeStatus::SUCCESS;
}

namespace {
// Starts a FollowPath waypoint string item: the path so far, then "frame: ".
void beginWaypoint(UwrtBtNode& n, std::ostringstream& out, const std::string& frame) {
    const std::string path = tryGetOptionalInput<std::string>(&n, "path", "");
    out << std::setprecision(10) << path;
    if(!path.empty() && path.back() != ';') {
        out << ";";
    }
    if(!path.empty()) {
        out << " ";
    }
    if(!frame.empty()) {
        out << frame << ": ";
    }
}

// The heading options (" | heading=..."), from the node's heading ports. look_x/y/z
// default to `look` (the arc centre for AddArc), or to look_frame's origin when set.
bool appendHeading(UwrtBtNode& n, std::ostringstream& out, const double look[3]) {
    const std::string lookFrame = tryGetOptionalInput<std::string>(&n, "look_frame", "");
    const std::string heading = tryGetOptionalInput<std::string>(&n, "heading", lookFrame.empty() ? "waypoint" : "look_at");
    if(heading != "waypoint" && heading != "path" && heading != "look_at") {
        RCLCPP_ERROR(n.rosNode()->get_logger(), "%s: heading must be waypoint, path or look_at, not \"%s\"",
            n.treeNode()->name().c_str(), heading.c_str());
        return false;
    }
    if(!lookFrame.empty() && heading != "look_at") {
        RCLCPP_ERROR(n.rosNode()->get_logger(), "%s: look_frame needs heading look_at, not \"%s\"",
            n.treeNode()->name().c_str(), heading.c_str());
        return false;
    }
    if(heading != "waypoint") {
        out << " | heading=" << heading;
    }
    if(heading == "look_at") {
        const double origin[3] = {0, 0, 0};
        const double* at = lookFrame.empty() ? look : origin;
        out << " | look_at=" << tryGetOptionalInput<double>(&n, "look_x", at[0]) << ","
            << tryGetOptionalInput<double>(&n, "look_y", at[1]) << ","
            << tryGetOptionalInput<double>(&n, "look_z", at[2]);
        if(!lookFrame.empty()) {
            out << " | look_frame=" << lookFrame;
        }
    }
    const double yawOffset = tryGetOptionalInput<double>(&n, "yaw_offset", 0), spin = tryGetOptionalInput<double>(&n, "spin", 0);
    if(yawOffset != 0) {
        out << " | yaw_offset=" << yawOffset;
    }
    if(spin != 0) {
        out << " | spin=" << spin;
    }
    const double spinRate = tryGetOptionalInput<double>(&n, "spin_rate", 0);
    if(spinRate != 0) {
        out << " | spin_rate=" << spinRate;
    }
    return true;
}
} // namespace

/**
 * @brief Appends one "frame: x,y,z,yaw;" waypoint to a FollowPath waypoint string, so a long
 * path can be built one readable node per waypoint. Optional heading ports choose how the
 * vehicle faces on the way there (see riptide_msgs2/PathSegment).
 *
 * @param n The BehaviorTree node.
 * @return BT::NodeStatus return status
 */
BT::NodeStatus addWaypoint(UwrtBtNode& n) {
    const std::string frame = tryGetOptionalInput<std::string>(&n, "frame", "");
    double
        x = tryGetRequiredInput<double>(&n, "x", 0),
        y = tryGetRequiredInput<double>(&n, "y", 0),
        z = tryGetRequiredInput<double>(&n, "z", 0),
        yaw = tryGetOptionalInput<double>(&n, "yaw", 0);

    std::ostringstream waypoint;
    beginWaypoint(n, waypoint, frame);
    waypoint << x << "," << y << "," << z << "," << yaw;
    const double noLook[3] = {0, 0, 0};
    if(!appendHeading(n, waypoint, noLook)) {
        return BT::NodeStatus::FAILURE;
    }
    waypoint << ";";

    postOutput<std::string>(&n, "out", waypoint.str());
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief Appends an arc about (cx, cy) to a FollowPath waypoint string: a straight line to the
 * point `radius` from the centre at `start_angle`, then `sweep` radians round (+ = counterclockwise
 * seen from above), ending at `end_radius` (a spiral if it differs) and depth z. Angles are from
 * the frame's +x axis.
 *
 * @param n The BehaviorTree node.
 * @return BT::NodeStatus return status
 */
BT::NodeStatus addArc(UwrtBtNode& n) {
    const std::string frame = tryGetOptionalInput<std::string>(&n, "frame", "");
    const double
        cx = tryGetRequiredInput<double>(&n, "cx", 0),
        cy = tryGetRequiredInput<double>(&n, "cy", 0),
        radius = tryGetRequiredInput<double>(&n, "radius", 1),
        startAngle = tryGetRequiredInput<double>(&n, "start_angle", 0),
        sweep = tryGetRequiredInput<double>(&n, "sweep", 0),
        z = tryGetRequiredInput<double>(&n, "z", 0),
        endRadius = tryGetOptionalInput<double>(&n, "end_radius", radius),
        yaw = tryGetOptionalInput<double>(&n, "yaw", 0);
    const bool approach = tryGetOptionalInput<std::string>(&n, "approach", "true") != "false";
    const double look[3] = {cx, cy, z};

    std::ostringstream arc;
    beginWaypoint(n, arc, frame);
    if(approach) {
        arc << cx + radius * cos(startAngle) << "," << cy + radius * sin(startAngle) << "," << z << "," << yaw;
        if(!appendHeading(n, arc, look)) {
            return BT::NodeStatus::FAILURE;
        }
        arc << "; ";
        if(!frame.empty()) {
            arc << frame << ": ";
        }
    }
    const double endAngle = startAngle + sweep;
    arc << cx + endRadius * cos(endAngle) << "," << cy + endRadius * sin(endAngle) << "," << z << "," << yaw
        << " | arc=" << cx << "," << cy << "," << sweep;
    if(!appendHeading(n, arc, look)) {
        return BT::NodeStatus::FAILURE;
    }
    arc << ";";

    postOutput<std::string>(&n, "out", arc.str());
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus checkBlackboardExists(UwrtBtNode& n) {
    std::string s = tryGetOptionalInput<std::string>(&n, "input", "");
    return s.empty() ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}


void registerSimpleUwrtAction(BT::BehaviorTreeFactory& factory, const std::string& id, const UWRTSimpleActionNode::TickFunctor& tickFunctor, BT::PortsList ports) {
    BT::NodeBuilder builder = [tickFunctor, id] (const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<UWRTSimpleActionNode>(name, tickFunctor, config);
    };

    BT::TreeNodeManifest manifest = {BT::NodeType::ACTION, id, std::move(ports), {}};
    factory.registerBuilder(manifest, builder);
}

/**
 * @brief Registers simple actions to be done by the BehaviorTree.
 * 
 * @param factory The factory to register with.
 */
void bulkRegisterSimpleActions(BT::BehaviorTreeFactory &factory) {
    registerSimpleUwrtAction(factory, "Info", printInfo, { BT::InputPort<std::string>("message") } );
    registerSimpleUwrtAction(factory, "Error", printError, { BT::InputPort<std::string>("message") } );

    /**
     * Basic action that calculates the distance between two points.
     */
    registerSimpleUwrtAction(factory, "CalculateDistance", calculateDistance,
        {
            UwrtInput("x1"),
            UwrtInput("y1"),
            UwrtInput("z1"),
            UwrtInput("x2"),
            UwrtInput("y2"),
            UwrtInput("z2"),
            UwrtOutput("dist")
        }
    );

    /**
     * Basic action that does math with two numbers (can add, subtract, multiply, or divide)
     */
    registerSimpleUwrtAction(factory, "Math", doMath, 
        {
            UwrtInput("a"),
            UwrtInput("b"),
            UwrtInput("operator"),
            UwrtOutput("out")
        }
    );

    registerSimpleUwrtAction(factory, "Format", format,
        {
            UwrtInput("format"),
            UwrtOutput("out")
        }
    );

    registerSimpleUwrtAction(factory, "HeadingToPoint", getHeadingToPoint,
        {
            UwrtInput("currX"),
            UwrtInput("currY"),
            UwrtInput("targX"),
            UwrtInput("targY"),
            UwrtOutput("heading")
        }
    );

    /**
     * Appends a waypoint to a FollowPath waypoint string. Leave path empty to start a new path.
     */
    registerSimpleUwrtAction(factory, "AddWaypoint", addWaypoint,
        {
            UwrtInput("path", "Waypoints so far (leave empty to start a new path)"),
            UwrtInput("frame", "Frame of this waypoint (default: FollowPath's frame)"),
            UwrtInput("x"),
            UwrtInput("y"),
            UwrtInput("z"),
            UwrtInput("yaw", "End yaw for heading waypoint (default 0)"),
            UwrtInput("heading", "waypoint (default: blend to yaw), path (face travel) or look_at"),
            UwrtInput("look_frame", "Face this TF frame, followed live as it moves (sets heading look_at)"),
            UwrtInput("look_x", "look_at point in frame, or in look_frame (default 0)"),
            UwrtInput("look_y", "look_at point in frame, or in look_frame (default 0)"),
            UwrtInput("look_z", "look_at point in frame, or in look_frame (default 0)"),
            UwrtInput("yaw_offset", "Added to path / look_at yaw (default 0)"),
            UwrtInput("spin", "Extra yaw over this leg, 6.283 = one turn (default 0)"),
            UwrtInput("spin_rate", "Steady spin on this leg, rad per metre (default 0)"),
            UwrtOutput("out", "Path with this waypoint appended")
        }
    );

    /**
     * Appends an arc (a line to its start, then round the centre) to a FollowPath waypoint string.
     */
    registerSimpleUwrtAction(factory, "AddArc", addArc,
        {
            UwrtInput("path", "Waypoints so far (leave empty to start a new path)"),
            UwrtInput("frame", "Frame of the centre and angles (default: FollowPath's frame)"),
            UwrtInput("cx", "Centre x in frame"),
            UwrtInput("cy", "Centre y in frame"),
            UwrtInput("radius"),
            UwrtInput("start_angle", "Where the arc starts, from the frame's +x (rad)"),
            UwrtInput("sweep", "How far round (rad, + = counterclockwise from above)"),
            UwrtInput("z", "Depth in frame"),
            UwrtInput("end_radius", "Radius at the end (default radius; differs = spiral)"),
            UwrtInput("approach", "false: start from the previous point, no line to the arc (default true)"),
            UwrtInput("yaw", "End yaw for heading waypoint (default 0)"),
            UwrtInput("heading", "waypoint (default), path (face travel) or look_at; also the approach line's"),
            UwrtInput("look_frame", "Face this TF frame, followed live as it moves (sets heading look_at)"),
            UwrtInput("look_x", "look_at point in frame (default: the centre), or in look_frame (default 0)"),
            UwrtInput("look_y", "look_at point in frame (default: the centre), or in look_frame (default 0)"),
            UwrtInput("look_z", "look_at point in frame (default: z), or in look_frame (default 0)"),
            UwrtInput("yaw_offset", "Added to path / look_at yaw (default 0)"),
            UwrtInput("spin", "Extra yaw over the arc, 6.283 = one turn (default 0)"),
            UwrtInput("spin_rate", "Steady spin on the arc, rad per metre (default 0)"),
            UwrtOutput("out", "Path with the arc appended")
        }
    );

    registerSimpleUwrtAction(factory, "CheckBlackboardExists", checkBlackboardExists,
        {
            UwrtInput("input")
        }
    );
}
