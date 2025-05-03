#include <modelec_strat/missions/go_home_mission.hpp>

namespace Modelec
{
    GoHomeMission::GoHomeMission(const std::shared_ptr<NavigationHelper>& nav) : status_(MissionStatus::READY), nav_(nav)
    {
    }

    void GoHomeMission::start(rclcpp::Node::SharedPtr node)
    {
        node_ = node;

        nav_->GoTo(400, 1600.0, -M_PI_2);

        status_ = MissionStatus::RUNNING;
    }

    void GoHomeMission::update()
    {
        if (nav_->HasArrived())
        {
            status_ = MissionStatus::DONE;
        }
    }

    MissionStatus GoHomeMission::getStatus() const
    {
        return status_;
    }
}
