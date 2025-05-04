#include <modelec_strat/missions/go_home_mission.hpp>

namespace Modelec
{
    GoHomeMission::GoHomeMission(const std::shared_ptr<NavigationHelper>& nav, const rclcpp::Time& start_time) : step_(GO_HOME), status_(MissionStatus::READY), nav_(nav), start_time_(start_time)
    {
    }

    void GoHomeMission::start(rclcpp::Node::SharedPtr node)
    {
        node_ = node;

        auto pos = nav_->GetHomePosition();
        home_point_ = Point(pos->x, pos->y, pos->theta);
        nav_->GoTo(home_point_.GetTakeBasePosition());

        status_ = MissionStatus::RUNNING;
    }

    void GoHomeMission::update()
    {
        if (!nav_->HasArrived()) return;

        switch (step_)
        {
            case GO_HOME:
                if ((node_->now() - start_time_).seconds() < 94)
                {
                    break;
                }

                nav_->GoTo(home_point_.GetTakePosition(0), true);

                step_ = GO_CLOSE;
                break;

            case GO_CLOSE:
                step_ = DONE;
                status_ = MissionStatus::DONE;

                break;
            default:
                break;
        }
    }

    void GoHomeMission::clear()
    {
    }

    MissionStatus GoHomeMission::getStatus() const
    {
        return status_;
    }
}
