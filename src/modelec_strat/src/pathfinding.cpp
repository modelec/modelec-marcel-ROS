#include <modelec_strat/pathfinding.hpp>

namespace Modelec {

    Pathfinding::Pathfinding()
    {
    }

    Pathfinding::Pathfinding(rclcpp::Node::SharedPtr node) : node_(std::move(node))
    {
    }

    rclcpp::Node::SharedPtr Pathfinding::getNode() const
    {
        return node_;
    }

    WaypointListMsg Pathfinding::FindPath(const PosMsg::SharedPtr& start, const PosMsg::SharedPtr& goal)
    {
        /* TODO - pathfinding
         *   This is a stub for the pathfinding algorithm. The pathfinding algorithm should
         */

        WaypointListMsg waypoints;
        waypoints.push_back(Waypoint(*start, 0));
        WaypointMsg middleW;
        middleW.x = 500;
        middleW.y = 1500;
        middleW.theta = 0;
        middleW.id = 1;
        middleW.is_end = false;
        waypoints.push_back(middleW);
        waypoints.push_back(Waypoint(*goal, 2, true));
        return waypoints;
    }

    Waypoint::Waypoint(const modelec_interfaces::msg::OdometryPos& pos, int index, bool isLast)
    {
        id = index;
        x = pos.x;
        y = pos.y;
        theta = pos.theta;
        is_end = isLast;
        reached = false;
    }

    Waypoint::Waypoint(const WaypointMsg& waypoint)
    {
        id = waypoint.id;
        x = waypoint.x;
        y = waypoint.y;
        theta = waypoint.theta;
        is_end = waypoint.is_end;
        reached = false;
    }

    WaypointMsg Waypoint::toMsg() const
    {
        return static_cast<OdometryAddWaypoint_<std::allocator<void>>>(*this);
    }
}
