#include <modelec_strat/navigation_helper.hpp>
#include <utility>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "../../modelec_utils/include/modelec_utils/config.hpp"

namespace Modelec {
    NavigationHelper::NavigationHelper()
    {
    }

    NavigationHelper::NavigationHelper(const rclcpp::Node::SharedPtr& node) : node_(node)
    {
        pathfinding_ = std::make_shared<Pathfinding>(node);

        factor_close_enemy_ = Config::get<float>("config.enemy.factor_close_enemy", -0.5f);

        waypoint_reach_sub_ = node_->create_subscription<WaypointReachMsg>(
            "odometry/waypoint_reach", 10,
            [this](const WaypointReachMsg::SharedPtr msg) {
                OnWaypointReach(msg);
            });

        waypoint_pub_ = node_->create_publisher<WaypointMsg>("odometry/add_waypoint", 100);

        pos_sub_ = node_->create_subscription<modelec_interfaces::msg::OdometryPos>(
            "odometry/position", 20,
            [this](const modelec_interfaces::msg::OdometryPos::SharedPtr msg) {
                OnPos(msg);
            });

        go_to_sub_ = node_->create_subscription<PosMsg>(
            "nav/go_to", 10, [this](const PosMsg::SharedPtr msg) {
                GoTo(msg, false, Pathfinding::FREE | Pathfinding::WALL);
            });

        enemy_pos_sub_ = node_->create_subscription<modelec_interfaces::msg::OdometryPos>(
            "enemy/position", 10,
            [this](const modelec_interfaces::msg::OdometryPos::SharedPtr msg)
            {
                OnEnemyPosition(msg);
            });

        std::string deposite_zone_path = ament_index_cpp::get_package_share_directory("modelec_strat") + "/data/deposite_zone.xml";
        if (!LoadDepositeZoneFromXML(deposite_zone_path))
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to load obstacles from XML");
        }
    }

    rclcpp::Node::SharedPtr NavigationHelper::GetNode() const
    {
        return node_;
    }

    std::shared_ptr<Pathfinding> NavigationHelper::GetPathfinding() const
    {
        return pathfinding_;
    }

    int NavigationHelper::GetTeamId() const
    {
        return team_id_;
    }

    void NavigationHelper::SendWaypoint() const
    {
        for (auto & w : waypoints_)
        {
            waypoint_pub_->publish(w.toMsg());
        }
    }

    void NavigationHelper::SendWaypoint(const std::vector<WaypointMsg>& waypoints) const
    {
        for (auto & w : waypoints)
        {
            waypoint_pub_->publish(w);
        }
    }

    void NavigationHelper::AddWaypoint(const PosMsg& pos, int index)
    {
        for (auto w = waypoints_.begin(); w != waypoints_.end(); ++w)
        {
            if (w->id == index)
            {
                waypoints_.insert(w, Waypoint(pos, index));

                while (w != waypoints_.end())
                {
                    w->id++;
                    ++w;
                }

                return;
            }
        }
        waypoints_.back().is_end = false;
        auto newWaypoint = Waypoint(pos, index);
        newWaypoint.is_end = true;
        waypoints_.emplace_back(newWaypoint);
    }

    void NavigationHelper::AddWaypoint(const WaypointMsg& waypoint)
    {
        for (auto w = waypoints_.begin(); w != waypoints_.end(); ++w)
        {
            if (w->id == waypoint.id)
            {
                waypoints_.insert(w, Waypoint(waypoint));

                while (w != waypoints_.end())
                {
                    w->id++;
                    ++w;
                }

                return;
            }
        }

        waypoints_.back().is_end = false;
        auto newWaypoint = Waypoint(waypoint);
        newWaypoint.is_end = true;
        waypoints_.emplace_back(newWaypoint);
    }

    void NavigationHelper::AddWaypoints(const std::initializer_list<PosMsg>& pos_list, int index)
    {
        for (auto w = waypoints_.begin(); w != waypoints_.end(); ++w)
        {
            if (w->id == index)
            {
                waypoints_.erase(w, waypoints_.end());

                for (const auto& pos : pos_list)
                {
                    waypoints_.emplace_back(pos, index);
                    index++;
                }
            }
        }
    }

    void NavigationHelper::AddWaypoints(const std::initializer_list<WaypointMsg>& waypoint_list)
    {
        int index = waypoint_list.begin()->id;
        for (auto w = waypoints_.begin(); w != waypoints_.end(); ++w)
        {
            if (w->id == index)
            {
                waypoints_.erase(w, waypoints_.end());

                for (const auto& pos : waypoint_list)
                {
                    waypoints_.emplace_back(pos);
                    index++;
                }
            }
        }
        waypoints_.back().is_end = false;

        for (auto & w : waypoint_list)
        {
            waypoints_.emplace_back(w);
        }
        waypoints_.back().is_end = true;
    }

    void NavigationHelper::SetWaypoints(const std::list<Waypoint>& waypoints)
    {
        waypoints_.clear();
        for (const auto& waypoint : waypoints)
        {
            waypoints_.emplace_back(waypoint);
        }
    }

    bool NavigationHelper::HasArrived() const
    {
        return waypoints_.back().reached;
    }

    bool NavigationHelper::RotateTo(const PosMsg::SharedPtr& pos)
    {
        double angle = std::atan2(pos->y - current_pos_->y, pos->x - current_pos_->x);

        if (std::abs(angle - current_pos_->theta) > M_PI / 3)
        {
            Rotate(angle);
            return true;
        }
        return false;
    }

    bool NavigationHelper::RotateTo(const Point& pos)
    {
        double angle = std::atan2(pos.y - current_pos_->y, pos.x - current_pos_->x);

        if (std::abs(angle - current_pos_->theta) > M_PI / 3)
        {
            Rotate(angle);
            return true;
        }
        return false;
    }

    void NavigationHelper::Rotate(double angle)
    {
        waypoints_.clear();

        WaypointMsg startAngle;
        startAngle.x = current_pos_->x;
        startAngle.y = current_pos_->y;
        startAngle.theta = angle;
        startAngle.id = 0;
        startAngle.is_end = true;

        waypoints_.emplace_back(startAngle);
        SendWaypoint();
    }

    int NavigationHelper::GoTo(const PosMsg::SharedPtr& goal, bool isClose, int collisionMask)
    {
        last_go_to_ = {goal, isClose, collisionMask};

        auto [res, wl] = FindPath(goal, isClose, collisionMask);

        if (wl.empty() || res != Pathfinding::FREE)
        {
            return res;
        }

        waypoints_.clear();

        for (auto & w : wl)
        {
            waypoints_.emplace_back(w);
        }

        SendWaypoint();

        return res;
    }

    int NavigationHelper::GoTo(int x, int y, double theta, bool isClose, int collisionMask)
    {
        PosMsg::SharedPtr goal = std::make_shared<PosMsg>();
        goal->x = x;
        goal->y = y;
        goal->theta = theta;
        return GoTo(goal, isClose, collisionMask);
    }

    int NavigationHelper::GoTo(const Point& goal, bool isClose, int collisionMask)
    {
        return GoTo(goal.x, goal.y, goal.theta, isClose, collisionMask);
    }

    int NavigationHelper::GoToRotateFirst(const PosMsg::SharedPtr& goal, bool isClose, int collisionMask)
    {
        last_go_to_ = {goal, isClose, collisionMask};

        auto [res, wl] = FindPath(goal, isClose, collisionMask);

        if (wl.empty() || res != Pathfinding::FREE)
        {
            return res;
        }

        auto p = Point(wl[0].x, wl[0].y, wl[0].theta);
        if (RotateTo(p))
        {
            await_rotate_ = true;

            send_back_waypoints_.clear();

            for (auto & w : wl)
            {
                send_back_waypoints_.emplace_back(w);
            }
        }
        else
        {
            waypoints_.clear();

            for (auto & w : wl)
            {
                waypoints_.emplace_back(w);
            }

            SendWaypoint();
        }

        return res;
    }

    int NavigationHelper::GoToRotateFirst(int x, int y, double theta, bool isClose, int collisionMask)
    {
        PosMsg::SharedPtr goal = std::make_shared<PosMsg>();
        goal->x = x;
        goal->y = y;
        goal->theta = theta;
        return GoToRotateFirst(goal, isClose, collisionMask);
    }

    int NavigationHelper::GoToRotateFirst(const Point& goal, bool isClose, int collisionMask)
    {
        return GoToRotateFirst(goal.x, goal.y, goal.theta, isClose, collisionMask);
    }

    int NavigationHelper::CanGoTo(const PosMsg::SharedPtr& goal, bool isClose, int collisionMask)
    {
        return FindPath(goal, isClose, collisionMask).first;
    }

    int NavigationHelper::CanGoTo(int x, int y, double theta, bool isClose, int collisionMask)
    {
        PosMsg::SharedPtr goal = std::make_shared<PosMsg>();
        goal->x = x;
        goal->y = y;
        goal->theta = theta;
        return CanGoTo(goal, isClose, collisionMask);
    }

    int NavigationHelper::CanGoTo(const Point& goal, bool isClose, int collisionMask)
    {
        return CanGoTo(goal.x, goal.y, goal.theta, isClose, collisionMask);
    }

    std::pair<int, WaypointListMsg> NavigationHelper::FindPath(const PosMsg::SharedPtr& goal, bool isClose,
                                                               int collisionMask)
    {
        return pathfinding_->FindPath(current_pos_, goal, isClose, collisionMask);
    }

    PosMsg::SharedPtr NavigationHelper::GetCurrentPos() const
    {
        return current_pos_;
    }

    bool NavigationHelper::LoadDepositeZoneFromXML(const std::string& filename)
    {
        tinyxml2::XMLDocument doc;
        if (doc.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to load obstacle XML file: %s", filename.c_str());
            return false;
        }

        tinyxml2::XMLElement* root = doc.FirstChildElement("Map");
        if (!root)
        {
            RCLCPP_ERROR(node_->get_logger(), "No <Obstacles> root element in file");
            return false;
        }

        for (tinyxml2::XMLElement* elem = root->FirstChildElement("DepositeZone");
             elem != nullptr;
             elem = elem->NextSiblingElement("DepositeZone"))
        {
            std::shared_ptr<DepositeZone> obs = std::make_shared<DepositeZone>(elem);
            deposite_zones_[obs->GetId()] = obs;
        }

        RCLCPP_INFO(node_->get_logger(), "Loaded %zu zone from XML", deposite_zones_.size());
        return true;
    }

    std::shared_ptr<DepositeZone> NavigationHelper::GetClosestDepositeZone(const PosMsg::SharedPtr& pos, int teamId, const std::vector<int>& blacklistedId)
    {
        // TODO : score
        std::shared_ptr<DepositeZone> closest_zone = nullptr;
        double score = std::numeric_limits<double>::max();
        auto posPoint = Point(pos->x, pos->y, pos->theta);
        auto enemyPos = Point(last_enemy_pos_.x, last_enemy_pos_.y, last_enemy_pos_.theta);

        for (const auto& [id, zone] : deposite_zones_)
        {
            if (zone->GetTeam() == teamId && zone->RemainingPotPos() > 0 && blacklistedId.end() == std::find(blacklistedId.begin(), blacklistedId.end(), id))
            {
                double distance = Point::distance(posPoint, zone->GetPosition());
                double enemy_distance = Point::distance(enemyPos, zone->GetPosition());
                double s = distance + enemy_distance * factor_close_enemy_;
                if (s < score)
                {
                    score = s;
                    closest_zone = zone;
                }
            }
        }

        return closest_zone;
    }

    PosMsg::SharedPtr NavigationHelper::GetHomePosition()
    {
        PosMsg::SharedPtr home = std::make_shared<PosMsg>();
        // TODO : handle Team Id
        if (team_id_ == YELLOW)
        {
            home->x = Config::get<int>("config.spawn.yellow@x", 0);
            home->y = Config::get<int>("config.spawn.yellow@y", 0);
            home->theta = Config::get<double>("config.spawn.yellow@theta", 0);
        }
        else
        {
            home->x = Config::get<int>("config.spawn.blue@x", 0);
            home->y = Config::get<int>("config.spawn.blue@y", 0);
            home->theta = Config::get<double>("config.spawn.blue@theta", 0);
        }
        return home;
    }

    void NavigationHelper::OnEnemyPosition(const modelec_interfaces::msg::OdometryPos::SharedPtr msg)
    {
        pathfinding_->OnEnemyPosition(msg);
        last_enemy_pos_ = *msg;

        if (EnemyOnPath(*msg))
        {
            RCLCPP_INFO(node_->get_logger(), "Enemy is blocking the path, replanning...");
            Replan();
        }
    }

    bool NavigationHelper::EnemyOnPath(const modelec_interfaces::msg::OdometryPos msg)
    {
        for (size_t i = -1; i + 1 < waypoints_.size(); ++i)
        {
            auto wp = i == -1 ? Waypoint(*current_pos_, -1, false) : waypoints_[i];
            auto next_wp = waypoints_[i + 1];
            if (DoesLineIntersectCircle(
                    Point(wp.x, wp.y, wp.theta),
                    Point(next_wp.x, next_wp.y, next_wp.theta),
                    Point(msg.x, msg.y, msg.theta), (pathfinding_->enemy_length_mm_ + pathfinding_->robot_length_mm_ + pathfinding_->enemy_margin_mm_) / 2.0f))
            {
                return true;
            }
        }

        return false;
    }

    bool NavigationHelper::DoesLineIntersectCircle(const Point& start, const Point& end, const Point& center, float radius)
    {
        float num = std::abs((end.y - start.y) * center.x - (end.x - start.x) * center.y + end.x * start.y - end.y * start.x);
        float den = std::sqrt((end.y - start.y) * (end.y - start.y) + (end.x - start.x) * (end.x - start.x));
        float dist = num / den;
        return dist < radius;
    }

    void NavigationHelper::Replan()
    {
        if (last_go_to_.goal)
        {
            if (GoTo(last_go_to_.goal, last_go_to_.isClose, last_go_to_.collisionMask) != Pathfinding::FREE)
            {
                if (GoTo(last_go_to_.goal, true, last_go_to_.collisionMask) != Pathfinding::FREE)
                {
                    // TODO : change to reset all the waypoints
                    GoTo(current_pos_, true, Pathfinding::FREE | Pathfinding::WALL | Pathfinding::OBSTACLE | Pathfinding::ENEMY);
                    // TODO : Handle case where no path is found
                }
            }
        }
    }

    void NavigationHelper::OnWaypointReach(const WaypointReachMsg::SharedPtr msg)
    {
        for (auto & waypoint : waypoints_)
        {
            if (waypoint.id == msg->id)
            {
                waypoint.reached = true;

                if (await_rotate_)
                {
                    await_rotate_ = false;

                    waypoints_.clear();
                    for (auto & w : send_back_waypoints_)
                    {
                        waypoints_.emplace_back(w);
                    }
                    SendWaypoint();
                }
            }
        }
    }

    void NavigationHelper::OnPos(const PosMsg::SharedPtr msg)
    {
        current_pos_ = msg;
        pathfinding_->SetCurrentPos(msg);
    }

}
