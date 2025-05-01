#include <modelec_strat/missions/promotion_mission.hpp>

namespace Modelec
{
    PromotionMission::PromotionMission(const std::shared_ptr<NavigationHelper>& nav) : step_(GO_TO_FRONT), status_(MissionStatus::READY), nav_(nav)
    {
    }

    void PromotionMission::start(rclcpp::Node::SharedPtr node)
    {
        node_ = node;

        status_ = MissionStatus::RUNNING;
    }

    void PromotionMission::update()
    {
    }

    MissionStatus PromotionMission::getStatus() const
    {
        return status_;
    }
}
