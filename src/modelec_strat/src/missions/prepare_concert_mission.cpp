#include <modelec_strat/missions/prepare_concert_mission.hpp>

namespace Modelec
{
    PrepareConcertMission::PrepareConcertMission(const std::shared_ptr<NavigationHelper>& nav) : step_(GO_TO_COLUMN),
        status_(MissionStatus::READY), nav_(nav)
    {
    }

    void PrepareConcertMission::start(rclcpp::Node::SharedPtr node)
    {
        column_ = nav_->getPathfinding()->GetObstacle(14);

        node_ = node;

        nav_->GoTo(775, 480, -M_PI_2);

        status_ = MissionStatus::RUNNING;
    }

    void PrepareConcertMission::update()
    {
        if (status_ != MissionStatus::RUNNING) return;

        if (!nav_->HasArrived()) return;

        switch (step_)
        {
        case GO_TO_COLUMN:
            nav_->GoTo(775, 460, -M_PI_2, true);

            step_ = GO_CLOSE_TO_COLUMN;
            break;
        case GO_CLOSE_TO_COLUMN:
            nav_->getPathfinding()->RemoveObstacle(column_.id());

            step_ = TAKE_COLUMN;
            break;
        case TAKE_COLUMN:
            {
                nav_->GoTo(2500, 875, 0);
            }

            step_ = GO_TO_PLATFORM;
            break;

        case GO_TO_PLATFORM:
            {
                nav_->GoTo(2700, 875, 0, true);
            }

            step_ = GO_CLOSE_TO_PLATFORM;
            break;

        case GO_CLOSE_TO_PLATFORM:
            {
                column_.setX(2925);
                column_.setY(875);
                column_.setTheta(0);
                nav_->getPathfinding()->AddObstacle(column_);
            }

            step_ = PLACE_PLATFORM;
            break;

        case PLACE_PLATFORM:
            step_ = DONE;
            status_ = MissionStatus::DONE;
            break;

        default:
            break;
        }
    }

    MissionStatus PrepareConcertMission::getStatus() const
    {
        return status_;
    }
}
