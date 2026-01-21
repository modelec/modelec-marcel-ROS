#include <modelec_strat/missions/free_mission.hpp>

#include "modelec_strat/action/base_action.hpp"

namespace Modelec {
    FreeMission::FreeMission(const std::shared_ptr<NavigationHelper>& nav, const std::shared_ptr<ActionExecutor>& action_executor)
     : step_(GO_TO_FREE), status_(MissionStatus::READY), nav_(nav), action_executor_(action_executor)  {
    }

    void FreeMission::Start(rclcpp::Node::SharedPtr node)
    {
        node_ = node;

        go_timeout_ = node_->now();

        status_ = MissionStatus::RUNNING;
        step_ = GO_TO_FREE;
    }

    void FreeMission::Update()
    {
        if (!action_executor_->IsActionDone())
        {
            return;
        }

        if (!nav_->HasArrived())
        {
            if ((node_->now() - go_timeout_).seconds() < 5)
            {
                // nav_->AskWaypoint();
                // return;
            }
            if ((node_->now() - go_timeout_).seconds() < 10)
            {
                return;
            }
        }

        switch (step_)
        {
        case GO_TO_FREE:
            {
                auto currPos = nav_->GetCurrentPos();

                auto dist = std::clamp(Point::distance(Point(currPos->x, currPos->y, currPos->theta),
                    nav_->GetClosestDepositeZone(nav_->GetCurrentPos())->GetPosition()), 0.0, 300.0);

                target_deposite_zone_ = nav_->GetClosestDepositeZone(nav_->GetCurrentPos(), {}, true);

                auto depoPoint = target_deposite_zone_->GetBestTakePosition(Point(currPos->x, currPos->y, currPos->theta));

                if (nav_->GoToRotateFirst(depoPoint.GetTakePosition(dist), true, Pathfinding::FREE))
                {
                    nav_->GoToRotateFirst(depoPoint.GetTakePosition(dist), true, Pathfinding::FREE | Pathfinding::OBSTACLE);
                }

                go_timeout_ = node_->now();
            }

            step_ = FREE;
            break;
        case DOWN:
            {
                action_executor_->Down(BaseAction::FRONT);
                deploy_time_ = node_->now();
            }

            step_ = FREE;
            break;
        case FREE:
            {
                action_executor_->Free({{0, true}, {1, true}, {2, true}, {3, true}});
                deploy_time_ = node_->now();

                auto obs = action_executor_->box_obstacles_[0];
                action_executor_->box_obstacles_[0] = nullptr;

                auto pos = nav_->GetCurrentPos();

                obs->SetPosition(
                    pos->x + 250 * cos(pos->theta),
                    pos->y + 250 * sin(pos->theta),
                    pos->theta);

                obs->SetAtObjective(true);

                nav_->GetPathfinding()->AddObstacle(obs);
            }

            step_ = UP;
            break;
        case UP:
            {
                action_executor_->Up(BaseAction::FRONT);
                deploy_time_ = node_->now();
            }

            step_ = GO_BACK;
            break;
        case GO_BACK:
            {
                nav_->GoTo(target_deposite_zone_->GetPosition().GetTakePosition(500), true, Pathfinding::FREE | Pathfinding::OBSTACLE);

                go_timeout_ = node_->now();
            }

            step_ = DONE;
            break;
        case DONE:
            {
                target_deposite_zone_->Validate(true);
            }

            status_ = MissionStatus::DONE;
            break;
        default:
            break;
        }
    }

    void FreeMission::Clear()
    {
    }

    MissionStatus FreeMission::GetStatus() const
    {
        return status_;
    }

}
