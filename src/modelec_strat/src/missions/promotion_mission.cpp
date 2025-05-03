#include <modelec_strat/missions/promotion_mission.hpp>

namespace Modelec
{
    PromotionMission::PromotionMission(const std::shared_ptr<NavigationHelper>& nav) : step_(GO_TO_FRONT), status_(MissionStatus::READY), nav_(nav)
    {
    }

    void PromotionMission::start(rclcpp::Node::SharedPtr node)
    {
        node_ = node;

        nav_->GoTo(1225, 152, M_PI_2, true);

        status_ = MissionStatus::RUNNING;
    }

    void PromotionMission::update()
    {
        if (status_ != MissionStatus::RUNNING) return;

        if (!nav_->HasArrived()) return;

        switch (step_)
        {
        case GO_TO_FRONT:
            // TODO deploy the banner here

            step_ = DEPLOY_BANNER;
            break;

        case DEPLOY_BANNER:
            nav_->GoTo(1225, 300, M_PI_2, true);

            step_ = GO_FORWARD;
            break;

        case GO_FORWARD:
            step_ = DONE;
            status_ = MissionStatus::DONE;
            break;

        default:
            break;
        }
    }

    MissionStatus PromotionMission::getStatus() const
    {
        return status_;
    }
}
