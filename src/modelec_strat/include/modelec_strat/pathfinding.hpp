#pragma once

#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/empty.hpp>

#include <tinyxml2.h>

#include <modelec_interfaces/msg/odometry_add_waypoint.hpp>
#include <modelec_interfaces/msg/odometry_waypoint_reach.hpp>
#include <modelec_interfaces/msg/odometry_pos.hpp>
#include <modelec_interfaces/msg/map.hpp>
#include <modelec_interfaces/srv/map.hpp>
#include <modelec_interfaces/srv/map_size.hpp>
#include <modelec_interfaces/msg/obstacle.hpp>


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

        Pathfinding(const rclcpp::Node::SharedPtr& node);

        rclcpp::Node::SharedPtr getNode() const;

        WaypointListMsg FindPath(const PosMsg::SharedPtr& start,
                      const PosMsg::SharedPtr& goal);

        bool LoadObstaclesFromXML(const std::string& filename);

    protected:
        void HandleMapRequest(
            const std::shared_ptr<modelec_interfaces::srv::Map::Request> request,
            const std::shared_ptr<modelec_interfaces::srv::Map::Response> response);

        void HandleMapSizeRequest(
            const std::shared_ptr<modelec_interfaces::srv::MapSize::Request> request,
            const std::shared_ptr<modelec_interfaces::srv::MapSize::Response> response);

        void HandleAskObstacleRequest(
            const std::shared_ptr<std_srvs::srv::Empty::Request> request,
            const std::shared_ptr<std_srvs::srv::Empty::Response> response);

    private:
        int robot_width_mm_;
        int robot_length_mm_;

        int grid_width_;
        int grid_height_;

        std::vector<std::vector<int>> grid_;

        std::map<int, modelec_interfaces::msg::Obstacle> obstacle_map_;

        rclcpp::Node::SharedPtr node_;

        rclcpp::Service<modelec_interfaces::srv::Map>::SharedPtr map_srv_;
        rclcpp::Service<modelec_interfaces::srv::MapSize>::SharedPtr map_size_srv_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr ask_obstacle_srv_;

        rclcpp::Publisher<modelec_interfaces::msg::Obstacle>::SharedPtr map_pub_;

    };

}
