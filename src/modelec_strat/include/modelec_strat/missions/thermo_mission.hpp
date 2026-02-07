#pragma once

#include <modelec_strat/missions/mission_base.hpp>
#include <modelec_strat/navigation_helper.hpp>
#include <modelec_strat/action_executor.hpp>

namespace Modelec {
    class ThermoMission : public Mission {
    public:
        ThermoMission(const std::shared_ptr<NavigationHelper>& nav,
                      const std::shared_ptr<ActionExecutor>& action_executor);

        void Start(rclcpp::Node::SharedPtr node) override;
        void Update() override;
        void Clear() override;
        MissionStatus GetStatus() const override;
        std::string GetName() const override { return "Thermo"; }

        static bool IsThermoDone;

    private:
        enum Step
        {
            GO_TO_THERMO,
            GO_TO_THERMO_CLOSE,
            ACTIVATE_THERMO,
            GO_TO_10,
            DEACTIVATE_THERMO,
            DONE,
        };

        std::array<Point, 2> thermo_positions_;

        std::shared_ptr<BoxObstacle> closestBox;
        MissionStatus status_;
        std::shared_ptr<NavigationHelper> nav_;
        std::shared_ptr<ActionExecutor> action_executor_;
        rclcpp::Node::SharedPtr node_;

        rclcpp::Time go_timeout_;
        rclcpp::Time deploy_time_;

        std::optional<rclcpp::Time> min_time_;

        rclcpp::Time last_ask_waypoint_time_;
    };
}