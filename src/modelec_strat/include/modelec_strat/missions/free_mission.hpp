#pragma once

#include <modelec_strat/missions/action_mission.hpp>
#include <modelec_strat/missions/move_mission.hpp>
#include <modelec_strat/missions/min_time_mission.hpp>

namespace Modelec {
    class FreeMission : public ActionMission, public MoveMission, public MinTimeMission {
    public:
        FreeMission(const std::shared_ptr<NavigationHelper>& nav,
                      const std::shared_ptr<ActionExecutor>& action_executor,
                      BaseAction::Side side = BaseAction::FRONT);

        void Start(const rclcpp::Node::SharedPtr& node) override;
        bool Update() override;
        void Clear() override;
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

        std::shared_ptr<DepositeZone> target_deposite_zone_;

        double angle_;
    };
}