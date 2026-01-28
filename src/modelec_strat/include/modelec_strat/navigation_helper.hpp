#pragma once

#include <rclcpp/rclcpp.hpp>
#include <modelec_interfaces/msg/odometry_waypoint.hpp>
#include <modelec_interfaces/msg/odometry_waypoints.hpp>
#include <modelec_interfaces/msg/odometry_pos.hpp>
#include <modelec_interfaces/msg/odometry_go_to.hpp>
#include <modelec_interfaces/msg/spawn.hpp>
#include <std_srvs/srv/empty.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>

#include "deposite_zone.hpp"
#include "pathfinding.hpp"

namespace Modelec
{
    class NavigationHelper
    {
    public:
        enum
        {
            YELLOW = 0,
            BLUE = 1,
        };

        NavigationHelper();

        NavigationHelper(const rclcpp::Node::SharedPtr& node);

        void ReInit();

        rclcpp::Node::SharedPtr GetNode() const;

        std::shared_ptr<Pathfinding> GetPathfinding() const;

        int GetTeamId() const;

        void Update();

        // void SendWaypoint() const;
        // void SendWaypoint(const std::vector<WaypointMsg>& waypoints) const;
        void SendWaypoint() const;
        void SendWaypoint(const std::vector<WaypointMsg>& waypoints) const;

        void SendWaypoints() const;
        void SendWaypoints(const std::vector<WaypointMsg>& waypoints) const;
        void SendWaypoints(const WaypointsMsg& waypoints) const;

        void SendGoTo();

        void AddWaypoint(const PosMsg& pos, int index);
        void AddWaypoint(const WaypointMsg& waypoint);

        void AddWaypoints(const std::initializer_list<PosMsg>& pos_list, int index);
        void AddWaypoints(const std::initializer_list<WaypointMsg>& waypoint_list);

        void SetWaypoints(const std::list<Waypoint>& waypoints);

        bool HasArrived() const;

        bool RotateTo(const PosMsg::SharedPtr& pos, bool front = true);
        bool RotateTo(const Point& pos, bool front = true);
        void Rotate(double angle);

        int GoTo(const PosMsg::SharedPtr& goal, bool isClose = false, int collisionMask = Pathfinding::FREE);
        int GoTo(int x, int y, double theta, bool isClose = false, int collisionMask = Pathfinding::FREE);
        int GoTo(const Point& goal, bool isClose = false, int collisionMask = Pathfinding::FREE);

        int GoToRotateFirst(const PosMsg::SharedPtr& goal, bool isClose = false, int collisionMask = Pathfinding::FREE, bool front = true);
        int GoToRotateFirst(int x, int y, double theta, bool isClose = false, int collisionMask = Pathfinding::FREE, bool front = true);
        int GoToRotateFirst(const Point& goal, bool isClose = false, int collisionMask = Pathfinding::FREE, bool front = true);

        int CanGoTo(const PosMsg::SharedPtr& goal, bool isClose = false, int collisionMask = Pathfinding::FREE);
        int CanGoTo(int x, int y, double theta, bool isClose = false, int collisionMask = Pathfinding::FREE);
        int CanGoTo(const Point& goal, bool isClose = false, int collisionMask = Pathfinding::FREE);

        std::pair<int, WaypointListMsg> FindPath(const PosMsg::SharedPtr& goal, bool isClose = false,
                                                 int collisionMask = Pathfinding::FREE);

        void SetPos(const PosMsg& pos);
        void SetPos(const Point& pos);
        void SetPos(int x, int y, double theta);

        PosMsg::SharedPtr GetCurrentPos() const;

        bool LoadDepositeZoneFromXML(const std::string& filename);

        std::shared_ptr<DepositeZone> GetClosestDepositeZone(const PosMsg::SharedPtr& pos,
                                                             const std::vector<int>& blacklistedId = {}, bool only_free = false);

        template <typename T,
          typename = std::enable_if_t<std::is_base_of<Obstacle, T>::value>>
        std::shared_ptr<T> GetClosestObstacle(const PosMsg::SharedPtr& pos) const;

        PosMsg::SharedPtr GetHomePosition();

        void OnEnemyPosition(const modelec_interfaces::msg::OdometryPos::SharedPtr msg);

        void OnEnemyPositionLongTime(const modelec_interfaces::msg::OdometryPos::SharedPtr msg);

        bool DoesLineIntersectCircle(const Point& start, const Point& end,
                                     const Point& center, float radius);

        bool EnemyOnPath(const modelec_interfaces::msg::OdometryPos msg);

        bool Replan(bool force = false);

        void SetTeamId(int id);

        void SetSpawn(const std::string& name);

        Point GetSpawn() const;

        void AskWaypoint();

    protected:
        void OnWaypointReach(const WaypointMsg::SharedPtr msg);

        void OnPos(const PosMsg::SharedPtr msg);

        void SetupSpawn();

        struct
        {
            PosMsg::SharedPtr goal;
            bool isClose;
            int collisionMask;
        } last_go_to_;

    private:
        rclcpp::Node::SharedPtr node_;

        std::shared_ptr<Pathfinding> pathfinding_;

        int team_id_ = YELLOW;
        std::map<std::string, Point> spawn_yellow_;
        std::map<std::string, Point> spawn_blue_;
        Point spawn_;

        float factor_close_enemy_ = 0;

        int enemy_emergency_distance_ = 0;

        bool last_was_close_enemy_ = false;

        std::vector<Waypoint> waypoints_;
        std::queue<Waypoint> waypoint_queue_;

        PosMsg::SharedPtr current_pos_;

        std::map<int, std::shared_ptr<DepositeZone>> deposite_zones_;

        rclcpp::Subscription<WaypointMsg>::SharedPtr waypoint_reach_sub_;
        rclcpp::Publisher<WaypointMsg>::SharedPtr waypoint_pub_;
        rclcpp::Publisher<WaypointsMsg>::SharedPtr waypoints_pub_;

        rclcpp::Subscription<modelec_interfaces::msg::OdometryGoTo>::SharedPtr go_to_sub_;
        rclcpp::Subscription<PosMsg>::SharedPtr pos_sub_;
        rclcpp::Publisher<PosMsg>::SharedPtr pos_pub_;

        rclcpp::Subscription<modelec_interfaces::msg::OdometryPos>::SharedPtr enemy_pos_sub_;
        rclcpp::Subscription<modelec_interfaces::msg::OdometryPos>::SharedPtr enemy_pos_long_time_sub_;

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_odo_pub_;

        modelec_interfaces::msg::OdometryPos last_enemy_pos_;
        bool has_enemy_ = false;

        bool await_rotate_ = false;
        std::vector<Waypoint> send_back_waypoints_;

        rclcpp::Publisher<modelec_interfaces::msg::Spawn>::SharedPtr spawn_pub_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr ask_spawn_srv_;

        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr odo_ask_waypoint_pub_;

    };

    template <typename T, typename>
    std::shared_ptr<T> NavigationHelper::GetClosestObstacle(const PosMsg::SharedPtr& pos) const
    {
        std::shared_ptr<T> closest_obstacle = nullptr;
        auto robotPos = Point(pos->x, pos->y, pos->theta);
        auto enemyPos = Point(last_enemy_pos_.x, last_enemy_pos_.y, last_enemy_pos_.theta);
        float distance = std::numeric_limits<float>::max();

        for (const auto& obstacle : GetPathfinding()->GetObstacles())
        {
            if (auto obs = std::dynamic_pointer_cast<T>(obstacle.second))
            {
                if (!obs->IsAtObjective())
                {
                    auto dist = Point::distance(robotPos, obs->GetPosition());

                    if (has_enemy_)
                    {
                        auto enemyDist = Point::distance(enemyPos, obs->GetPosition());
                        dist *= (1.0f + factor_close_enemy_ * std::exp(-enemyDist / enemy_emergency_distance_));
                    }

                    if (dist < distance)
                    {
                        distance = dist;
                        closest_obstacle = obs;
                    }
                }
            }
        }

        return closest_obstacle;
    }
}