#pragma once

#include <rclcpp/rclcpp.hpp>
#include <modelec_interfaces/msg/odometry_add_waypoint.hpp>
#include <modelec_interfaces/msg/odometry_waypoint_reach.hpp>
#include <modelec_interfaces/msg/odometry_pos.hpp>

#include "pathfinding.hpp"

namespace Modelec {

    class NavigationHelper {
    public:
        NavigationHelper();

        NavigationHelper(const rclcpp::Node::SharedPtr& node);

        rclcpp::Node::SharedPtr getNode() const;

        std::shared_ptr<Pathfinding> getPathfinding() const;

        void SendWaypoint() const;
        void SendWaypoint(const std::vector<WaypointMsg> &waypoints) const;

        void AddWaypoint(const PosMsg &pos, int index);
        void AddWaypoint(const WaypointMsg &waypoint);

        void AddWaypoints(const std::initializer_list<PosMsg> &pos_list, int index);
        void AddWaypoints(const std::initializer_list<WaypointMsg> &waypoint_list);

        void SetWaypoints(const std::list<Waypoint> &waypoints);

        bool HasArrived() const;

        void GoTo(const PosMsg::SharedPtr &goal, bool isClose = false);
        void GoTo(int x, int y, double theta, bool isClose = false);

        WaypointListMsg FindPath(const PosMsg::SharedPtr &goal, bool isClose = false);

    protected:
        void OnWaypointReach(const WaypointReachMsg::SharedPtr msg);

        void OnPos(const PosMsg::SharedPtr msg);

    private:
        rclcpp::Node::SharedPtr node_;

        std::shared_ptr<Pathfinding> pathfinding_;

        std::list<Waypoint> waypoints_;

        PosMsg::SharedPtr current_pos_;

        rclcpp::Subscription<WaypointReachMsg>::SharedPtr waypoint_reach_sub_;
        rclcpp::Publisher<WaypointMsg>::SharedPtr waypoint_pub_;

        rclcpp::Subscription<PosMsg>::SharedPtr go_to_sub_;
        rclcpp::Subscription<PosMsg>::SharedPtr pos_sub_;
    };
}