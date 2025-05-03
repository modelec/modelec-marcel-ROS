#include <modelec_strat/missions/prepare_concert_mission.hpp>
#include <modelec_strat/obstacle/column.hpp>

namespace Modelec
{
    PrepareConcertMission::PrepareConcertMission(const std::shared_ptr<NavigationHelper>& nav) : step_(GO_TO_COLUMN),
        status_(MissionStatus::READY), nav_(nav)
    {
    }

    void PrepareConcertMission::start(rclcpp::Node::SharedPtr node)
    {
        if (auto col = nav_->getPathfinding()->GetClosestColumn(nav_->GetCurrentPos()))
        {
            column_ = col;
        } else
        {
            status_ = MissionStatus::FAILED;
            return;
        }

        node_ = node;

        auto pos = column_->position().GetTakeBasePosition();

        nav_->GoTo(pos.x, pos.y, pos.theta);

        status_ = MissionStatus::RUNNING;
    }

    void PrepareConcertMission::update()
    {
        if (status_ != MissionStatus::RUNNING) return;

        if (!nav_->HasArrived()) return;

        switch (step_)
        {
        case GO_TO_COLUMN:
            {
                auto pos = column_->position().GetTakeClosePosition();
                nav_->GoTo(pos.x, pos.y, pos.theta, true);
            }

            step_ = GO_CLOSE_TO_COLUMN;
            break;
        case GO_CLOSE_TO_COLUMN:
            nav_->getPathfinding()->RemoveObstacle(column_->id());

            step_ = TAKE_COLUMN;
            break;
        case TAKE_COLUMN:
            {
                auto pos = column_->position().GetTakeBasePosition();
                nav_->GoTo(pos.x, pos.y, pos.theta, true);
            }

            step_ = GO_BACK;
            break;
        case GO_BACK:
            {
                closestDepoZone_ = nav_->GetClosestDepositeZone(nav_->GetCurrentPos(), 0);
                closestDepoZonePoint_ = closestDepoZone_->GetNextPotPos();
                auto p = closestDepoZonePoint_.GetTakeBasePosition();
                nav_->GoTo(p.x, p.y, p.theta);
            }

            step_ = GO_TO_PLATFORM;
            break;
        case GO_TO_PLATFORM:
            {
                auto p = closestDepoZonePoint_.GetTakeClosePosition();
                nav_->GoTo(p.x, p.y, p.theta, true);
            }

            step_ = GO_CLOSE_TO_PLATFORM;
            break;

        case GO_CLOSE_TO_PLATFORM:
            {
                column_->setX(closestDepoZonePoint_.x);
                column_->setY(closestDepoZonePoint_.y);
                column_->setTheta(closestDepoZonePoint_.theta);
                column_->SetAtObjective(true);
                nav_->getPathfinding()->AddObstacle(column_);
            }

            step_ = PLACE_PLATFORM;
            break;

        case PLACE_PLATFORM:
            {
                auto p = closestDepoZonePoint_.GetTakeBasePosition();
                nav_->GoTo(p.x, p.y, p.theta, true);
            }

            step_ = GO_BACK_2;
            break;

        case GO_BACK_2:
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
