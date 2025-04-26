#include <modelec_strat/navigation_helper.hpp>
#include <utility>

namespace Modelec {
    NavigationHelper::NavigationHelper()
    {
    }

    NavigationHelper::NavigationHelper(const rclcpp::Node::SharedPtr& node) : node_(node)
    {
        pathfinding_ = std::make_unique<Pathfinding>(node);

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
    }

    rclcpp::Node::SharedPtr NavigationHelper::getNode() const
    {
        return node_;
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

    void NavigationHelper::GoTo(const PosMsg::SharedPtr& goal)
    {
        SendWaypoint(FindPath(goal));
    }

    WaypointListMsg NavigationHelper::FindPath(const PosMsg::SharedPtr& goal)
    {
        return pathfinding_->FindPath(current_pos_, goal);
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
    }

}
