#include <modelec_strat/missions/prepare_concert_mission.hpp>

namespace Modelec
{
    PrepareConcertMission::PrepareConcertMission(const std::shared_ptr<NavigationHelper>& nav) : step_(GO_TO_COLUMN), status_(MissionStatus::READY), nav_(nav)
    {
    }

    void PrepareConcertMission::start(rclcpp::Node::SharedPtr node)
    {
        node_ = node;

        

        status_ = MissionStatus::RUNNING;
    }

    void PrepareConcertMission::update()
    {
        if (status_ != MissionStatus::RUNNING) return;

        if (!nav_->HasArrived()) return;

        switch (step_)
        {
        case GO_TO_COLUMN:
            step_ = TAKE_COLUMN;
            break;

        case TAKE_COLUMN:
            step_ = GO_TO_PLATFORM;
            break;

        case GO_TO_PLATFORM:
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
