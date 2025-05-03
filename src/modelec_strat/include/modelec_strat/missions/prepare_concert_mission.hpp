#pragma once

#include <modelec_strat/missions/mission_base.hpp>
#include <modelec_strat/navigation_helper.hpp>

namespace Modelec {

    class PrepareConcertMission : public Mission {
    public:
        PrepareConcertMission(const std::shared_ptr<NavigationHelper>& nav);

        void start(rclcpp::Node::SharedPtr node) override;
        void update() override;
        MissionStatus getStatus() const override;
        std::string name() const override { return "PrepareConcert"; }

    private:
        enum Step {
            GO_TO_COLUMN,
            GO_CLOSE_TO_COLUMN,
            TAKE_COLUMN,
            GO_TO_PLATFORM,
            GO_CLOSE_TO_PLATFORM,
            PLACE_PLATFORM,
            DONE
        } step_;

        MissionStatus status_;
        std::shared_ptr<NavigationHelper> nav_;
        rclcpp::Node::SharedPtr node_;

        Obstacle column_;
    };

}
