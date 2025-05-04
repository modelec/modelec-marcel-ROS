#pragma once

#include <modelec_strat/missions/mission_base.hpp>
#include <modelec_strat/navigation_helper.hpp>

namespace Modelec
{
    /*
     * Banderole mission
     *
     */
    class PromotionMission : public Mission
    {
    public:
        PromotionMission(const std::shared_ptr<NavigationHelper>& nav);

        void start(rclcpp::Node::SharedPtr node) override;
        void update() override;
        void clear() override;
        MissionStatus getStatus() const override;
        std::string name() const override { return "Promotion"; }

        enum Step {
            GO_TO_FRONT,
            DEPLOY_BANNER,
            GO_FORWARD,
            DONE
        } step_;

        MissionStatus status_;
        std::shared_ptr<NavigationHelper> nav_;
        rclcpp::Node::SharedPtr node_;
    };
}
