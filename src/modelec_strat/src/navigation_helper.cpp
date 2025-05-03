#include <modelec_strat/navigation_helper.hpp>
#include <utility>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace Modelec {
    NavigationHelper::NavigationHelper()
    {
    }

    NavigationHelper::NavigationHelper(const rclcpp::Node::SharedPtr& node) : node_(node)
    {
        pathfinding_ = std::make_shared<Pathfinding>(node);

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
                GoTo(msg);
            });

        std::string deposite_zone_path = ament_index_cpp::get_package_share_directory("modelec_strat") + "/data/deposite_zone.xml";
        if (!LoadDepositeZoneFromXML(deposite_zone_path))
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to load obstacles from XML");
        }
    }

    rclcpp::Node::SharedPtr NavigationHelper::getNode() const
    {
        return node_;
    }

    std::shared_ptr<Pathfinding> NavigationHelper::getPathfinding() const
    {
        return pathfinding_;
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

    void NavigationHelper::GoTo(const PosMsg::SharedPtr& goal, bool isClose)
    {
        waypoints_.clear();

        auto wml = FindPath(goal, isClose);

        for (auto & w : wml)
        {
            waypoints_.emplace_back(w);
        }

        SendWaypoint();
    }

    void NavigationHelper::GoTo(int x, int y, double theta, bool isClose)
    {
        PosMsg::SharedPtr goal = std::make_shared<PosMsg>();
        goal->x = x;
        goal->y = y;
        goal->theta = theta;
        GoTo(goal, isClose);
    }

    WaypointListMsg NavigationHelper::FindPath(const PosMsg::SharedPtr& goal, bool isClose)
    {
        return pathfinding_->FindPath(current_pos_, goal, isClose);
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

        RCLCPP_INFO(node_->get_logger(), "Loaded %zu obstacles from XML", deposite_zones_.size());
        return true;
    }

    std::shared_ptr<DepositeZone> NavigationHelper::GetClosestDepositeZone(const PosMsg::SharedPtr& pos, int teamId)
    {
        std::shared_ptr<DepositeZone> closest_zone = nullptr;
        double min_distance = std::numeric_limits<double>::max();
        auto posPoint = Point(pos->x, pos->y, pos->theta);
        for (const auto& zone : deposite_zones_)
        {
            if (zone.second->GetTeam() == teamId)
            {
                double distance = Point::distance(posPoint, zone.second->GetPosition());
                if (distance < min_distance)
                {
                    min_distance = distance;
                    closest_zone = zone.second;
                }
            }
        }

        return closest_zone;
    }

    void NavigationHelper::OnWaypointReach(const WaypointReachMsg::SharedPtr msg)
    {
        for (auto & waypoint : waypoints_)
        {
            if (waypoint.id == msg->id)
            {
                waypoint.reached = true;
            }
        }
    }

    void NavigationHelper::OnPos(const PosMsg::SharedPtr msg)
    {
        current_pos_ = msg;
        pathfinding_->SetCurrentPos(msg);
    }

}
