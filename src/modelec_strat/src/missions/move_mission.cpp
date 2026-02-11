#include <modelec_strat/missions/move_mission.hpp>
#include <modelec_strat/navigation_helper.hpp>
#include <modelec_utils/config.hpp>

namespace Modelec {

    double MoveMission::MIN_ASK_WAYPOINT = 3.0;
    double MoveMission::ASK_WAYPOINT_INTERVAL = 2.0;
    double MoveMission::GO_TIMEOUT = 10.0;
    bool MoveMission::IsInit = false;

    MoveMission::MoveMission(const std::shared_ptr<NavigationHelper>& nav)
        : nav_(nav)
    {
    }

    void MoveMission::InitConfig()
    {
        MIN_ASK_WAYPOINT = Config::get<double>("config.mission.move.min_ask_waypoint.s", 3.0);
        ASK_WAYPOINT_INTERVAL = Config::get<double>("config.mission.move.ask_waypoint_interval.s", 2.0);
        GO_TIMEOUT = Config::get<double>("config.mission.move.go_timeout.s", 10.0);

        IsInit = true;
    }

    void MoveMission::Start(rclcpp::Node::SharedPtr node)
    {
        Mission::Start(node);

        if (!IsInit)
        {
            InitConfig();
        }

        go_timeout_ = node->now();
        last_ask_waypoint_time_ = node->now();
    }

    bool MoveMission::Update()
    {
        if (!nav_->HasArrived())
        {
            auto now = node_->now();
            double elapsed_total = (now - go_timeout_).seconds();
            double elapsed_since_last_ask = (now - last_ask_waypoint_time_).seconds();

            if (elapsed_total > MIN_ASK_WAYPOINT && elapsed_since_last_ask > ASK_WAYPOINT_INTERVAL)
            {
                nav_->AskWaypoint();
                last_ask_waypoint_time_ = now;
                return false;
            }

            if (elapsed_total < GO_TIMEOUT)
            {
                return false;
            }
        }

        return true;
    }
}