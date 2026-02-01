#include <modelec_strat/missions/thermo_mission.hpp>

#include "modelec_strat/action/base_action.hpp"

namespace Modelec {

    bool ThermoMission::IsThermoDone = false;

    ThermoMission::ThermoMission(const std::shared_ptr<NavigationHelper>& nav,
        const std::shared_ptr<ActionExecutor>& action_executor)
     : status_(MissionStatus::READY), nav_(nav), action_executor_(action_executor)
    {
    }

    void ThermoMission::Start(rclcpp::Node::SharedPtr node)
    {
        node_ = node;

        go_timeout_ = node_->now();
        deploy_time_ = node_->now();
        last_ask_waypoint_time_ = node_->now();

        status_ = MissionStatus::RUNNING;

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

    void ThermoMission::Update()
    {
        if (!action_executor_->IsActionDone())
        {
            return;
        }

        if (!nav_->HasArrived())
        {
            if ((node_->now() - go_timeout_).seconds() < 2 && (node_->now() - last_ask_waypoint_time_).seconds() > 1)
            {
                nav_->AskWaypoint();
                last_ask_waypoint_time_ = node_->now();
                return;
            }
            if ((node_->now() - go_timeout_).seconds() < 10)
            {
                return;
            }
        }

        if (min_time_.has_value())
        {
            if ((node_->now() - min_time_.value()).seconds() < 0.1)
            {
                return;
            }
            else
            {
                min_time_.reset();
            }
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
                    break;
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
                    break;
                }

                go_timeout_ = node_->now();
            }
            break;
        case ACTIVATE_THERMO:
            {
                RCLCPP_INFO(node_->get_logger(), "Activating thermo");
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
                    break;
                }
            }
            break;
        case DEACTIVATE_THERMO:
            {
                RCLCPP_INFO(node_->get_logger(), "Deactivating thermo");
                action_executor_->ActivateThermo(nav_->GetTeamId() == NavigationHelper::YELLOW ? BaseAction::RIGHT : BaseAction::LEFT, false);
                deploy_time_ = node_->now();
            }
            break;
        case DONE:
            {
                action_executor_->SendPoint(10);
                IsThermoDone = true;
            }

            status_ = MissionStatus::DONE;
            break;
        default:
            break;
        }
    }

    void ThermoMission::Clear()
    {
    }

    MissionStatus ThermoMission::GetStatus() const
    {
        return status_;
    }

}
