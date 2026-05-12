#include <modelec_strat/missions/thermo_mission.hpp>

#include "modelec_strat/action/base_action.hpp"

namespace Modelec {

    bool ThermoMission::IsThermoDone = false;

    ThermoMission::ThermoMission(const std::shared_ptr<NavigationHelper>& nav,
        const std::shared_ptr<ActionExecutor>& action_executor) :
        Mission(MissionStatus::READY), ActionMission(action_executor), MoveMission(nav)
    {
    }

    void ThermoMission::Start(const rclcpp::Node::SharedPtr& node)
    {
        ActionMission::Start(node);
        MoveMission::Start(node);

        status_ = MissionStatus::RUNNING;

        mission_score_ = Config::get<int>("config.mission_score.thermo", 0);

        thermo_positions_ = nav_->GetThermoPositions();

        std::queue<int> empty;
        std::swap(steps_, empty);

        steps_.push(GO_TO_THERMO);
        steps_.push(GO_TO_THERMO_CLOSE);
        steps_.push(ACTIVATE_THERMO);
        steps_.push(GO_TO_10);
        steps_.push(DEACTIVATE_THERMO);
        steps_.push(DONE);
    }

    bool ThermoMission::Update()
    {
        if (!ActionMission::Update() || !MoveMission::Update())
        {
            return false;
        }

        auto step_ = steps_.front();
        steps_.pop();

        switch (step_)
        {
        case GO_TO_THERMO:
            {
                auto start = thermo_positions_[0].GetTakePosition(CLOSE_DISTANCE, M_PI_2);

                if (nav_->GoToRotateFirst(start, true, Pathfinding::FREE | Pathfinding::WALL, true) != Pathfinding::FREE)
                {
                    status_ = MissionStatus::FAILED;
                    return false;
                }

                go_timeout_ = node_->now();
            }
            break;
        case GO_TO_THERMO_CLOSE:
            {
                auto start = thermo_positions_[0];

                if (nav_->GoToRotateFirst(start, true, Pathfinding::FREE | Pathfinding::WALL, true) != Pathfinding::FREE)
                {
                    status_ = MissionStatus::FAILED;
                    return false;
                }

                go_timeout_ = node_->now();
            }
            break;
        case ACTIVATE_THERMO:
            {
                action_executor_->ActivateThermo(nav_->GetTeamId() == NavigationHelper::YELLOW ? BaseAction::RIGHT : BaseAction::LEFT, true);
                deploy_time_ = node_->now();
            }
            break;
        case GO_TO_10:
            {
                auto pos = thermo_positions_[1];

                if (nav_->GoToRotateFirst(pos, true, Pathfinding::FREE | Pathfinding::WALL, true) != Pathfinding::FREE)
                {
                    status_ = MissionStatus::FAILED;
                    return false;
                }
            }
            break;
        case DEACTIVATE_THERMO:
            {
                action_executor_->ActivateThermo(nav_->GetTeamId() == NavigationHelper::YELLOW ? BaseAction::RIGHT : BaseAction::LEFT, false);
                deploy_time_ = node_->now();
            }
            break;
        case DONE:
            {
                action_executor_->SendPoint(mission_score_);
                IsThermoDone = true;
            }

            status_ = MissionStatus::DONE;
            break;
        default:
            break;
        }

        return true;
    }

    void ThermoMission::Clear()
    {
    }

}
