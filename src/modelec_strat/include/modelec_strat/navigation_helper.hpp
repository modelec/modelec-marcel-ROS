#pragma once

#include <rclcpp/rclcpp.hpp>
#include <modelec_interfaces/msg/odometry_add_waypoint.hpp>
#include <modelec_interfaces/msg/odometry_waypoint_reach.hpp>
#include <modelec_interfaces/msg/odometry_pos.hpp>
#include <modelec_interfaces/msg/odometry_go_to.hpp>
#include <modelec_interfaces/msg/spawn.hpp>
#include <std_srvs/srv/empty.hpp>

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

        void SendWaypoint() const;
        void SendWaypoint(const std::vector<WaypointMsg>& waypoints) const;

        void AddWaypoint(const PosMsg& pos, int index);
        void AddWaypoint(const WaypointMsg& waypoint);

        void AddWaypoints(const std::initializer_list<PosMsg>& pos_list, int index);
        void AddWaypoints(const std::initializer_list<WaypointMsg>& waypoint_list);

        void SetWaypoints(const std::list<Waypoint>& waypoints);

        bool HasArrived() const;

        bool RotateTo(const PosMsg::SharedPtr& pos);
        bool RotateTo(const Point& pos);
        void Rotate(double angle);

        int GoTo(const PosMsg::SharedPtr& goal, bool isClose = false, int collisionMask = Pathfinding::FREE);
        int GoTo(int x, int y, double theta, bool isClose = false, int collisionMask = Pathfinding::FREE);
        int GoTo(const Point& goal, bool isClose = false, int collisionMask = Pathfinding::FREE);

        int GoToRotateFirst(const PosMsg::SharedPtr& goal, bool isClose = false, int collisionMask = Pathfinding::FREE);
        int GoToRotateFirst(int x, int y, double theta, bool isClose = false, int collisionMask = Pathfinding::FREE);
        int GoToRotateFirst(const Point& goal, bool isClose = false, int collisionMask = Pathfinding::FREE);

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

        std::shared_ptr<DepositeZone> GetClosestDepositeZone(const PosMsg::SharedPtr& pos, int teamId,
                                                             const std::vector<int>& blacklistedId = {});

        PosMsg::SharedPtr GetHomePosition();

        void OnEnemyPosition(const modelec_interfaces::msg::OdometryPos::SharedPtr msg);
        void OnEnemyPositionClose(const modelec_interfaces::msg::OdometryPos::SharedPtr msg);

        void OnEnemyPositionLongTime(const modelec_interfaces::msg::OdometryPos::SharedPtr msg);

        bool DoesLineIntersectCircle(const Point& start, const Point& end,
                                     const Point& center, float radius);

        bool EnemyOnPath(const modelec_interfaces::msg::OdometryPos msg);

        bool Replan(bool force = true);

        void SetTeamId(int id);

        void SetSpawn(const std::string& name);

        Point GetSpawnYellow() const;
        Point GetSpawnBlue() const;
        Point GetSpawn() const;

    protected:
        void OnWaypointReach(const WaypointReachMsg::SharedPtr msg);

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

        bool last_was_close_enemy_ = false;

        std::vector<Waypoint> waypoints_;

        PosMsg::SharedPtr current_pos_;

        std::map<int, std::shared_ptr<DepositeZone>> deposite_zones_;

        rclcpp::Subscription<WaypointReachMsg>::SharedPtr waypoint_reach_sub_;
        rclcpp::Publisher<WaypointMsg>::SharedPtr waypoint_pub_;

        rclcpp::Subscription<modelec_interfaces::msg::OdometryGoTo>::SharedPtr go_to_sub_;
        rclcpp::Subscription<PosMsg>::SharedPtr pos_sub_;
        rclcpp::Publisher<PosMsg>::SharedPtr pos_pub_;

        rclcpp::Subscription<modelec_interfaces::msg::OdometryPos>::SharedPtr enemy_pos_sub_;
        rclcpp::Subscription<modelec_interfaces::msg::OdometryPos>::SharedPtr close_enemy_pos_sub_;
        rclcpp::Subscription<modelec_interfaces::msg::OdometryPos>::SharedPtr enemy_pos_long_time_sub_;

        modelec_interfaces::msg::OdometryPos last_enemy_pos_;

        bool await_rotate_ = false;
        std::vector<Waypoint> send_back_waypoints_;

        rclcpp::Publisher<modelec_interfaces::msg::Spawn>::SharedPtr spawn_pub_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr ask_spawn_srv_;

    };
}
