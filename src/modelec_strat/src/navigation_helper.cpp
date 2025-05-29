#include <modelec_strat/navigation_helper.hpp>
#include <utility>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "../../modelec_utils/include/modelec_utils/config.hpp"

namespace Modelec
{
    NavigationHelper::NavigationHelper()
    {
    }

    NavigationHelper::NavigationHelper(const rclcpp::Node::SharedPtr& node) : node_(node)
    {
        pathfinding_ = std::make_shared<Pathfinding>(node);

        factor_close_enemy_ = Config::get<float>("config.enemy.factor_close_enemy", -0.5f);

        SetupSpawn();

        waypoint_reach_sub_ = node_->create_subscription<WaypointReachMsg>(
            "odometry/waypoint_reach", 10,
            [this](const WaypointReachMsg::SharedPtr msg)
            {
                OnWaypointReach(msg);
            });

        waypoint_pub_ = node_->create_publisher<WaypointMsg>("odometry/add_waypoint", 100);

        pos_sub_ = node_->create_subscription<modelec_interfaces::msg::OdometryPos>(
            "odometry/position", 20,
            [this](const modelec_interfaces::msg::OdometryPos::SharedPtr msg)
            {
                OnPos(msg);
            });

        pos_pub_ = node_->create_publisher<PosMsg>("odometry/set_position", 10);

        go_to_sub_ = node_->create_subscription<modelec_interfaces::msg::OdometryGoTo>(
            "nav/go_to", 10, [this](const modelec_interfaces::msg::OdometryGoTo::SharedPtr msg)
            {
                GoTo(msg->x, msg->y, msg->theta, msg->close, msg->mask);
            });

        enemy_pos_sub_ = node_->create_subscription<modelec_interfaces::msg::OdometryPos>(
            "enemy/position", 10,
            [this](const modelec_interfaces::msg::OdometryPos::SharedPtr msg)
            {
                OnEnemyPosition(msg);
            });

        close_enemy_pos_sub_ = node_->create_subscription<modelec_interfaces::msg::OdometryPos>(
            "enemy/position/emergency", 10,
            [this](const modelec_interfaces::msg::OdometryPos::SharedPtr msg)
            {
                OnEnemyPositionClose(msg);
            });

        enemy_pos_long_time_sub_ = node_->create_subscription<modelec_interfaces::msg::OdometryPos>(
            "/enemy/long_time", 10,
            [this](const modelec_interfaces::msg::OdometryPos::SharedPtr msg)
            {
                OnEnemyPositionLongTime(msg);
            });

        start_odo_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/odometry/start", 10);

        std::string deposite_zone_path = ament_index_cpp::get_package_share_directory("modelec_strat") +
            "/data/deposite_zone.xml";
        if (!LoadDepositeZoneFromXML(deposite_zone_path))
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to load obstacles from XML");
        }

        spawn_pub_ = node_->create_publisher<modelec_interfaces::msg::Spawn>("nav/spawn", 10);

        ask_spawn_srv_ = node->create_service<std_srvs::srv::Empty>(
            "/nav/ask_spawn", [this](const std_srvs::srv::Empty::Request::SharedPtr,
                                      const std_srvs::srv::Empty::Response::SharedPtr)
            {
                for (auto& ys : spawn_yellow_)
                {
                    auto s = modelec_interfaces::msg::Spawn();
                    s.team_id = YELLOW;
                    s.name = ys.first;
                    s.x = ys.second.x;
                    s.y = ys.second.y;
                    s.theta = ys.second.theta;

                    spawn_pub_->publish(s);
                }

                for (auto& bs : spawn_blue_)
                {
                    auto s = modelec_interfaces::msg::Spawn();
                    s.team_id = BLUE;
                    s.name = bs.first;
                    s.x = bs.second.x;
                    s.y = bs.second.y;
                    s.theta = bs.second.theta;

                    spawn_pub_->publish(s);
                }
            });

        odo_get_pos_pub_ = node_->create_publisher<std_msgs::msg::Empty>(
            "odometry/get/pos", 30);

        last_odo_get_pos_time_ = node_->now();
    }

    void NavigationHelper::ReInit()
    {
        SetPos(spawn_);
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

    void NavigationHelper::Update()
    {
        if ((node_->now() - last_odo_get_pos_time_).seconds() > 1)
        {
            last_odo_get_pos_time_ = node_->now();
            RCLCPP_INFO(node_->get_logger(), "Requesting current position from odometry");
            std_msgs::msg::Empty empty_msg;
            odo_get_pos_pub_->publish(empty_msg);
        }
    }

    void NavigationHelper::SendGoTo()
    {
        while (!waypoint_queue_.empty())
        {
            waypoint_queue_.pop();
        }

        for (auto w : waypoints_)
        {
            w.id = 0;
            w.is_end = true;
            waypoint_queue_.push(w);
        }

        auto w = waypoint_queue_.front().ToMsg();
        RCLCPP_INFO(node_->get_logger(), "Sending waypoint: x: %d, y: %d, theta: %f, id: %d",
                     w.x, w.y, w.theta, w.id);
        waypoint_pub_->publish(w);
        waypoint_queue_.pop();
    }

/*    void NavigationHelper::SendWaypoint() const
    {
        for (auto& w : waypoints_)
        {
            waypoint_pub_->publish(w.ToMsg());
        }
    }

    void NavigationHelper::SendWaypoint(const std::vector<WaypointMsg>& waypoints) const
    {
        for (auto& w : waypoints)
        {
            waypoint_pub_->publish(w);
        }
    }
*/

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

        for (auto& w : waypoint_list)
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
        if (waypoints_.empty())
        {
            return true;
        }
        return waypoint_queue_.empty() && waypoints_.back().reached;
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
        // SendWaypoint();
        SendGoTo();
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

        for (auto& w : wl)
        {
            waypoints_.emplace_back(w);
        }

        // SendWaypoint();
        SendGoTo();

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

            for (auto& w : wl)
            {
                send_back_waypoints_.emplace_back(w);
            }
        }
        else
        {
            waypoints_.clear();

            for (auto& w : wl)
            {
                waypoints_.emplace_back(w);
            }

            // SendWaypoint();
            SendGoTo();
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

    void NavigationHelper::SetPos(const PosMsg& pos)
    {
        pos_pub_->publish(pos);
    }

    void NavigationHelper::SetPos(const Point& pos)
    {
        RCLCPP_INFO(node_->get_logger(), "Set position to x: %d, y: %d, theta: %f", pos.x, pos.y, pos.theta);

        PosMsg msg;
        msg.x = pos.x;
        msg.y = pos.y;
        msg.theta = pos.theta;
        SetPos(msg);

        PosMsg::SharedPtr goal = std::make_shared<PosMsg>();
        goal->x = pos.x;
        goal->y = pos.y;
        goal->theta = pos.theta;
        OnPos(goal);
    }

    void NavigationHelper::SetPos(int x, int y, double theta)
    {
        SetPos({x, y, theta});
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

    std::shared_ptr<DepositeZone> NavigationHelper::GetClosestDepositeZone(
        const PosMsg::SharedPtr& pos, int teamId, const std::vector<int>& blacklistedId)
    {
        std::shared_ptr<DepositeZone> closest_zone = nullptr;
        double score = std::numeric_limits<double>::max();
        auto posPoint = Point(pos->x, pos->y, pos->theta);
        auto enemyPos = Point(last_enemy_pos_.x, last_enemy_pos_.y, last_enemy_pos_.theta);

        for (const auto& [id, zone] : deposite_zones_)
        {
            if (zone->GetTeam() == teamId && zone->RemainingPotPos() > 0 && blacklistedId.end() == std::find(
                blacklistedId.begin(), blacklistedId.end(), id))
            {
                auto zonePoint = zone->GetNextPotPos().GetTakeBasePosition();
                double distance = Point::distance(posPoint, zonePoint);
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
        if (team_id_ == YELLOW)
        {
            home->x = Config::get<int>("config.home.yellow@x", 0);
            home->y = Config::get<int>("config.home.yellow@y", 0);
            home->theta = Config::get<double>("config.home.yellow@theta", 0);
        }
        else
        {
            home->x = Config::get<int>("config.home.blue@x", 0);
            home->y = Config::get<int>("config.home.blue@y", 0);
            home->theta = Config::get<double>("config.home.blue@theta", 0);
        }
        return home;    }

    void NavigationHelper::OnEnemyPosition(const modelec_interfaces::msg::OdometryPos::SharedPtr msg)
    {
        pathfinding_->OnEnemyPosition(msg);
        last_enemy_pos_ = *msg;

        if (last_was_close_enemy_)
        {
            if (Point::distance(Point(msg->x, msg->y, msg->theta), Point(current_pos_->x, current_pos_->y, current_pos_->theta)) > 600)
            {
                RCLCPP_INFO(node_->get_logger(), "Enemy was close try to replanning...");
                if (Replan(false))
                {
                    last_was_close_enemy_ = false;
                }
            }

            return;
        }

        if (HasArrived())
        {
            RCLCPP_DEBUG(node_->get_logger(), "No path to replanning, ignoring enemy position");
            return;
        }

        if (EnemyOnPath(*msg))
        {
            RCLCPP_INFO(node_->get_logger(), "Enemy is blocking the path, replanning...");
            Replan();
        }
    }

    void NavigationHelper::OnEnemyPositionClose(const modelec_interfaces::msg::OdometryPos::SharedPtr msg)
    {

        if (!last_was_close_enemy_)
        {
            RCLCPP_INFO(node_->get_logger(), "Enemy is close, replanning...");

            last_was_close_enemy_ = true;

            pathfinding_->OnEnemyPosition(msg);
            last_enemy_pos_ = *msg;

            std_msgs::msg::Bool start_odo_msg;
            start_odo_msg.data = false;
            start_odo_pub_->publish(start_odo_msg);

            /*waypoints_.clear();

            Waypoint w(*msg, -1, false);

            waypoints_.emplace_back(w);

            SendGoTo();*/
        }
    }

    void NavigationHelper::OnEnemyPositionLongTime(const modelec_interfaces::msg::OdometryPos::SharedPtr msg)
    {
        pathfinding_->OnEnemyPositionLongTime(msg);

        Point enemy_pos(msg->x, msg->y, msg->theta);
        for (auto& [id, zone] : deposite_zones_)
        {
            auto zonePos = zone->GetPosition();
            if (Point::distance(enemy_pos, zonePos) < pathfinding_->robot_width_mm_ + (zone->GetWidth() / 2) +
                pathfinding_->enemy_margin_mm_)
            {
                std::shared_ptr<Obstacle> obs = std::make_shared<Obstacle>(
                    id, zonePos.x, zonePos.y, zonePos.theta, zone->GetWidth(), zone->GetHeight(), "enemy-zone");
                pathfinding_->AddObstacle(obs);
            }
        }
    }

    bool NavigationHelper::EnemyOnPath(const modelec_interfaces::msg::OdometryPos msg)
    {
        if (!current_pos_) {
            return false;
        }

        auto curr = Waypoint(*current_pos_, -1, false);
        std::vector<Waypoint> waypointsList;
        waypointsList.push_back(curr);

        for (auto& waypoint : waypoints_)
        {
            if (waypoint.reached)
            {
                continue;
            }
            waypointsList.push_back(waypoint);
        }

        for (size_t i = 0; i + 1 < waypointsList.size(); ++i)
        {
            auto wp = waypointsList[i];
            auto next_wp = waypointsList[i + 1];
            if (DoesLineIntersectCircle(
                Point(wp.x, wp.y, wp.theta),
                Point(next_wp.x, next_wp.y, next_wp.theta),
                Point(msg.x, msg.y, msg.theta),
                (pathfinding_->enemy_length_mm_ + pathfinding_->robot_length_mm_ + pathfinding_->enemy_margin_mm_) /
                2.0f))
            {
                return true;
            }
        }

        return false;
    }

    bool NavigationHelper::DoesLineIntersectCircle(const Point& start, const Point& end, const Point& center,
                                                   float radius)
    {
        float num = std::abs(
            (end.y - start.y) * center.x - (end.x - start.x) * center.y + end.x * start.y - end.y * start.x);
        float den = std::sqrt((end.y - start.y) * (end.y - start.y) + (end.x - start.x) * (end.x - start.x));
        float dist = num / den;
        return dist < radius;
    }

    bool NavigationHelper::Replan(bool force)
    {
        if (last_go_to_.goal)
        {
            if (GoTo(last_go_to_.goal, last_go_to_.isClose, last_go_to_.collisionMask) != Pathfinding::FREE)
            {
                if (GoTo(last_go_to_.goal, true, last_go_to_.collisionMask) != Pathfinding::FREE)
                {
                    if (!force) return false;

                    if (GoTo(current_pos_, true,
                             Pathfinding::FREE | Pathfinding::WALL | Pathfinding::OBSTACLE | Pathfinding::ENEMY) !=
                        Pathfinding::FREE)
                    {
                        return false;
                    }
                }
            }
        }

        return true;
    }

    void NavigationHelper::SetTeamId(int id)
    {
        team_id_ = id;
    }

    void NavigationHelper::SetSpawn(const std::string& name)
    {
        switch (team_id_)
        {
        case YELLOW:
            SetPos(spawn_yellow_[name]);
            spawn_ = spawn_yellow_[name];
            break;
        case BLUE:
            SetPos(spawn_blue_[name]);
            spawn_ = spawn_blue_[name];
            break;
        default:
            SetPos(spawn_yellow_[name]);
            spawn_ = spawn_yellow_[name];
            break;
        }
    }

    Point NavigationHelper::GetSpawn() const
    {
        return spawn_;
    }

    void NavigationHelper::OnWaypointReach(const WaypointReachMsg::SharedPtr)
    {
        /*for (auto& waypoint : waypoints_)
        {
            if (waypoint.id == msg->id)
            {
                waypoint.reached = true;

                if (await_rotate_)
                {
                    await_rotate_ = false;

                    waypoints_.clear();
                    for (auto& w : send_back_waypoints_)
                    {
                        waypoints_.emplace_back(w);
                    }
                    SendWaypoint();
                }
            }
        }*/

        RCLCPP_INFO(node_->get_logger(), "Waypoint reached, id: %d", waypoint_queue_.front().id);

        if (await_rotate_)
        {
            await_rotate_ = false;

            waypoints_.clear();
            for (auto& w : send_back_waypoints_)
            {
                waypoints_.emplace_back(w);
            }
            // SendWaypoint();
            SendGoTo();
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "Waypoint reached 2");

            if (!waypoint_queue_.empty())
            {
                waypoint_pub_->publish(waypoint_queue_.front().ToMsg());
                waypoint_queue_.pop();

                waypoints_[waypoints_.size() - waypoint_queue_.size() - 1].reached = true;
            }
            else
            {
                waypoints_.back().reached = true;
            }
        }
    }

    void NavigationHelper::OnPos(const PosMsg::SharedPtr msg)
    {
        current_pos_ = msg;
        pathfinding_->SetCurrentPos(msg);
    }

    void NavigationHelper::SetupSpawn()
    {
        spawn_yellow_["top"] = Point(
            Config::get<int>("config.spawn.yellow.top@x"),
            Config::get<int>("config.spawn.yellow.top@y"),
            Config::get<double>("config.spawn.yellow.top@theta")
        );

        spawn_yellow_["side"] = Point(
            Config::get<int>("config.spawn.yellow.side@x"),
            Config::get<int>("config.spawn.yellow.side@y"),
            Config::get<double>("config.spawn.yellow.side@theta")
        );

        spawn_yellow_["bottom"] = Point(
            Config::get<int>("config.spawn.yellow.bottom@x"),
            Config::get<int>("config.spawn.yellow.bottom@y"),
            Config::get<double>("config.spawn.yellow.bottom@theta")
        );

        spawn_blue_["top"] = Point(
            Config::get<int>("config.spawn.blue.top@x"),
            Config::get<int>("config.spawn.blue.top@y"),
            Config::get<double>("config.spawn.blue.top@theta")
        );

        spawn_blue_["side"] = Point(
            Config::get<int>("config.spawn.blue.side@x"),
            Config::get<int>("config.spawn.blue.side@y"),
            Config::get<double>("config.spawn.blue.side@theta")
        );

        spawn_blue_["bottom"] = Point(
            Config::get<int>("config.spawn.blue.bottom@x"),
            Config::get<int>("config.spawn.blue.bottom@y"),
            Config::get<double>("config.spawn.blue.bottom@theta")
        );
    }
}
