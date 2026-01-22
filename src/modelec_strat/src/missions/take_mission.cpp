#include <modelec_strat/missions/take_mission.hpp>

#include "modelec_strat/action/base_action.hpp"

namespace Modelec {
    TakeMission::TakeMission(const std::shared_ptr<NavigationHelper>& nav, const std::shared_ptr<ActionExecutor>& action_executor)
     : step_(GO_TO_TAKE), status_(MissionStatus::READY), nav_(nav), action_executor_(action_executor)  {
    }

    void TakeMission::Start(rclcpp::Node::SharedPtr node)
    {
        node_ = node;

        go_timeout_ = node_->now();

        status_ = MissionStatus::RUNNING;
        step_ = GO_TO_TAKE;
    }

    void TakeMission::Update()
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
        case GO_TO_TAKE:
            {

                closestBox = nav_->GetClosestObstacle<BoxObstacle>(nav_->GetCurrentPos());

                action_executor_->box_obstacles_[0] = closestBox;

                auto pos = closestBox->GetOptimizedGetPos(nav_->GetCurrentPos()).GetTakeBasePosition();

                if (nav_->GoToRotateFirst(pos, false, Pathfinding::FREE | Pathfinding::WALL))
                {
                    if (nav_->GoToRotateFirst(pos, true, Pathfinding::FREE | Pathfinding::WALL))
                    {
                        nav_->GoToRotateFirst(pos, true, Pathfinding::FREE | Pathfinding::WALL | Pathfinding::OBSTACLE);
                    }
                }

                go_timeout_ = node_->now();
            }

            step_ = GO_TO_TAKE_CLOSE;
            break;
        case GO_TO_TAKE_CLOSE:
            {
                auto pos = closestBox->GetOptimizedGetPos(nav_->GetCurrentPos()).GetTakeClosePosition();

                nav_->GoToRotateFirst(pos, true, Pathfinding::FREE | Pathfinding::WALL | Pathfinding::OBSTACLE);

                go_timeout_ = node_->now();
            }

            step_ = DOWN;
            break;
        case DOWN:
            {
                action_executor_->Down(BaseAction::FRONT);
                deploy_time_ = node_->now();
            }

            step_ = TAKE;
            break;
        case TAKE:
            {
                action_executor_->Take({{0, true}, {1, true}, {2, true}, {3, true}});
                deploy_time_ = node_->now();
            }

            step_ = UP;
            break;
        case UP:
            {
                action_executor_->Up(BaseAction::FRONT);
                deploy_time_ = node_->now();
            }

            step_ = DONE;
            break;
        case DONE:
            nav_->GetPathfinding()->RemoveObstacle(closestBox->GetId());
            status_ = MissionStatus::DONE;
            break;
        default:
            break;
        }
    }

    void TakeMission::Clear()
    {
    }

    MissionStatus TakeMission::GetStatus() const
    {
        return status_;
    }

}
