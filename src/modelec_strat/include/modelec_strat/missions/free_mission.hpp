#pragma once

#include <modelec_strat/missions/mission_base.hpp>
#include <modelec_strat/navigation_helper.hpp>
#include <modelec_strat/action_executor.hpp>

namespace Modelec {
    class FreeMission : public Mission {
    public:
        FreeMission(const std::shared_ptr<NavigationHelper>& nav,
                      const std::shared_ptr<ActionExecutor>& action_executor,
                      BaseAction::Side side = BaseAction::FRONT);

        void Start(rclcpp::Node::SharedPtr node) override;
        void Update() override;
        void Clear() override;
        MissionStatus GetStatus() const override;
        std::string GetName() const override { return "Free"; }

    private:
        enum Step
        {
            GO_TO_FREE,
            CHECK_BOX,
            DOWN,
            FREE_FIRST,
            ROTATE_ARM,
            FREE_OTHER,
            UP,
            GO_BACK,
            DONE,
        };

        BaseAction::Side side_;

        MissionStatus status_;
        std::shared_ptr<NavigationHelper> nav_;
        std::shared_ptr<ActionExecutor> action_executor_;
        rclcpp::Node::SharedPtr node_;

        std::shared_ptr<DepositeZone> target_deposite_zone_;

        double angle_;

        rclcpp::Time go_timeout_;
        rclcpp::Time deploy_time_;

        std::optional<rclcpp::Time> min_time_;

        rclcpp::Time last_ask_waypoint_time_;
    };
}