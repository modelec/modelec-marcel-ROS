#pragma once

#include <rclcpp/rclcpp.hpp>

#include <modelec_interfaces/msg/odometry_add_waypoint.hpp>
#include <modelec_interfaces/msg/odometry_waypoint_reach.hpp>
#include <modelec_interfaces/msg/odometry_pos.hpp>

namespace Modelec {

    using WaypointMsg = modelec_interfaces::msg::OdometryAddWaypoint;
    using WaypointReachMsg = modelec_interfaces::msg::OdometryWaypointReach;
    using PosMsg = modelec_interfaces::msg::OdometryPos;
    using WaypointListMsg = std::vector<WaypointMsg>;

    class Waypoint : public WaypointMsg
    {
    public:
        bool reached = false;

        Waypoint(const PosMsg& pos, int index, bool isLast = false);

        explicit Waypoint(const WaypointMsg& waypoint);

        WaypointMsg toMsg() const;
    };

    /*
     * - Pathfinding DStar Lite
     *
     *
     *
     */

    class Pathfinding {
    public:
        Pathfinding();

        Pathfinding(rclcpp::Node::SharedPtr node);

        rclcpp::Node::SharedPtr getNode() const;

        WaypointListMsg FindPath(const PosMsg::SharedPtr& start,
                      const PosMsg::SharedPtr& goal);

    private:
        rclcpp::Node::SharedPtr node_;
    };

}