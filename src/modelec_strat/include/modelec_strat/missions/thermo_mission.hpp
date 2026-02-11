#pragma once

#include <modelec_strat/missions/action_mission.hpp>
#include <modelec_strat/missions/move_mission.hpp>

namespace Modelec {
    class ThermoMission : public ActionMission, public MoveMission {
    public:
        ThermoMission(const std::shared_ptr<NavigationHelper>& nav,
                      const std::shared_ptr<ActionExecutor>& action_executor);

        void Start(rclcpp::Node::SharedPtr node) override;
        bool Update() override;
        void Clear() override;
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
    };
}