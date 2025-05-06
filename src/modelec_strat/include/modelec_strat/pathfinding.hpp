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

#include <modelec_strat/obstacle/obstacle.hpp>

#include "obstacle/column.hpp"


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

    class Pathfinding {
    public:
        enum
        {
            FREE = 1 << 0,
            WALL = 1 << 1,
            OBSTACLE = 1 << 2,
            ENEMY = 1 << 3,
        };

        int grid_width_ = 0;
        int grid_height_ = 0;
        int map_width_mm_ = 0;
        int map_height_mm_ = 0;
        int robot_width_mm_ = 0;
        int robot_length_mm_ = 0;
        int enemy_width_mm_ = 0;
        int enemy_length_mm_ = 0;
        int enemy_margin_mm_ = 0;
        int margin_mm_ = 0;
        float factor_close_enemy_ = 0;

        Pathfinding();

        Pathfinding(const rclcpp::Node::SharedPtr& node);

        rclcpp::Node::SharedPtr getNode() const;

        std::pair<int, WaypointListMsg> FindPath(const PosMsg::SharedPtr& start,
                                                 const PosMsg::SharedPtr& goal, bool isClose = false,
                                                 int collisionMask = FREE);

        //void SetStartAndGoal(const PosMsg::SharedPtr& start, const PosMsg::SharedPtr& goal);

        bool LoadObstaclesFromXML(const std::string& filename);

        void SetCurrentPos(const PosMsg::SharedPtr& pos);

        std::shared_ptr<Obstacle> GetObstacle(int id) const;

        void RemoveObstacle(int id);

        void AddObstacle(const std::shared_ptr<Obstacle>& obstacle);

        template <typename T,
          typename = std::enable_if_t<std::is_base_of<Obstacle, T>::value>>
        std::shared_ptr<T> GetClosestObstacle(const PosMsg::SharedPtr& pos) const;

        std::shared_ptr<ColumnObstacle> GetClosestColumn(const PosMsg::SharedPtr& pos, const std::vector<int>& blacklistedId = {});

        void OnEnemyPosition(const modelec_interfaces::msg::OdometryPos::SharedPtr msg);

        void OnEnemyPositionLongTime(const modelec_interfaces::msg::OdometryPos::SharedPtr msg);

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

        //bool EnemyOnPath(const modelec_interfaces::msg::OdometryPos& enemy_pos);
        //void Replan();

        bool TestCollision(int x, int y, int collisionMask = FREE);

    private:
        rclcpp::Node::SharedPtr node_;

        std::vector<std::vector<int>> grid_;

        std::map<int, std::shared_ptr<Obstacle>> obstacle_map_;

        PosMsg::SharedPtr current_start_;
        PosMsg::SharedPtr current_goal_;
        WaypointListMsg current_waypoints_;

        modelec_interfaces::msg::OdometryPos last_enemy_pos_;
        bool has_enemy_pos_ = false;

        rclcpp::Subscription<modelec_interfaces::msg::OdometryPos>::SharedPtr enemy_pos_sub_;
        rclcpp::Publisher<WaypointMsg>::SharedPtr waypoint_pub_;

        rclcpp::Subscription<modelec_interfaces::msg::Obstacle>::SharedPtr obstacle_add_sub_;
        rclcpp::Subscription<modelec_interfaces::msg::Obstacle>::SharedPtr obstacle_remove_sub_;

        rclcpp::Publisher<modelec_interfaces::msg::Obstacle>::SharedPtr obstacle_add_pub_;
        rclcpp::Publisher<modelec_interfaces::msg::Obstacle>::SharedPtr obstacle_remove_pub_;

        rclcpp::Service<modelec_interfaces::srv::Map>::SharedPtr map_srv_;
        rclcpp::Service<modelec_interfaces::srv::MapSize>::SharedPtr map_size_srv_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr ask_obstacle_srv_;
    };

    template <typename T, typename>
    std::shared_ptr<T> Pathfinding::GetClosestObstacle(const PosMsg::SharedPtr& pos) const
    {
        std::shared_ptr<T> closest_obstacle = nullptr;
        auto robotPos = Point(pos->x, pos->y, pos->theta);
        float distance = std::numeric_limits<float>::max();

        for (const auto& obstacle : obstacle_map_) {
            if (auto obs = std::dynamic_pointer_cast<T>(obstacle.second)) {
                auto dist = Point::distance(robotPos, obs->position());
                if (dist < distance) {
                    distance = dist;
                    closest_obstacle = obs;
                }
            }
        }

        return closest_obstacle;
    }
}
