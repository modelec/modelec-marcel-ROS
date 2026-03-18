#pragma once

#include <modelec_strat/missions/action_mission.hpp>
#include <modelec_strat/missions/min_time_mission.hpp>
#include <modelec_strat/missions/move_mission.hpp>

namespace Modelec {
    class TakeMission : public ActionMission, public MoveMission {
    public:
        TakeMission(const std::shared_ptr<NavigationHelper>& nav,
                      const std::shared_ptr<ActionExecutor>& action_executor,
                      BaseAction::Side side = BaseAction::FRONT);

        void Start(const rclcpp::Node::SharedPtr& node) override;
        bool Update() override;
        void Clear() override;
        std::string GetName() const override { return "Take"; }

    private:
        enum Step
        {
            GO_TO_TAKE,
            GO_TO_TAKE_CLOSE,
            DOWN,
            TAKE,
            UP,
            DONE,
        };

        BaseAction::Side side_;

        std::shared_ptr<BoxObstacle> closestBox;
    };
}
