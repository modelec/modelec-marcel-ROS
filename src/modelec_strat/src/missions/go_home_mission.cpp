#include <modelec_strat/missions/go_home_mission.hpp>

namespace Modelec
{
    GoHomeMission::GoHomeMission(const std::shared_ptr<NavigationHelper>& nav, const rclcpp::Time& start_time) : step_(ROTATE_TO_HOME), status_(MissionStatus::READY), nav_(nav), start_time_(start_time)
    {
    }

    void GoHomeMission::start(rclcpp::Node::SharedPtr node)
    {
        node_ = node;

        auto pos = nav_->GetHomePosition();
        home_point_ = Point(pos->x, pos->y, pos->theta);
        if (nav_->CanGoTo(home_point_.GetTakeBasePosition()) != Pathfinding::FREE)
        {
            if (nav_->CanGoTo(home_point_.GetTakeBasePosition(), true) != Pathfinding::FREE)
            {
                status_ = MissionStatus::FAILED;
                return;
            }
        }
        nav_->RotateTo(home_point_);

        status_ = MissionStatus::RUNNING;
    }

    void GoHomeMission::update()
    {
        if (!nav_->HasArrived()) return;

        switch (step_)
        {
            case ROTATE_TO_HOME:
                {
                    if (nav_->GoTo(home_point_.GetTakeBasePosition()) != Pathfinding::FREE)
                    {
                        if (nav_->GoTo(home_point_.GetTakeBasePosition(), true) != Pathfinding::FREE)
                        {
                            status_ = MissionStatus::FAILED;
                            return;
                        }
                    }
                }

                step_ = GO_HOME;
                break;
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
