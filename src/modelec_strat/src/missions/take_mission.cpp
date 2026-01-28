#include <modelec_strat/missions/take_mission.hpp>

#include "modelec_strat/action/base_action.hpp"

namespace Modelec {
    TakeMission::TakeMission(const std::shared_ptr<NavigationHelper>& nav,
        const std::shared_ptr<ActionExecutor>& action_executor,
        BaseAction::Front front)
     : front_(front), status_(MissionStatus::READY), nav_(nav), action_executor_(action_executor)
    {
    }

    void TakeMission::Start(rclcpp::Node::SharedPtr node)
    {
        node_ = node;

        go_timeout_ = node_->now();
        deploy_time_ = node_->now();
        last_ask_waypoint_time_ = node_->now();

        status_ = MissionStatus::RUNNING;

        std::queue<int> empty;
        std::swap(steps_, empty);

        steps_.push(GO_TO_TAKE);
        steps_.push(GO_TO_TAKE_CLOSE);
        steps_.push(DOWN);
        steps_.push(TAKE);
        steps_.push(UP);
        steps_.push(DONE);
    }

    void TakeMission::Update()
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
        case GO_TO_TAKE:
            {

                closestBox = nav_->GetClosestObstacle<BoxObstacle>(nav_->GetCurrentPos());

                if (closestBox == nullptr)
                {
                    status_ = MissionStatus::FAILED;
                    break;
                }

                action_executor_->box_obstacles_[front_] = closestBox;

                auto pos = closestBox->GetOptimizedGetPos(nav_->GetCurrentPos()).GetTakeBasePosition();
                pos.theta += (front_ == BaseAction::FRONT ? 0 : M_PI);

                if (nav_->GoToRotateFirst(pos, false, Pathfinding::FREE | Pathfinding::WALL, front_ == BaseAction::FRONT) != Pathfinding::FREE)
                {
                    if (nav_->GoToRotateFirst(pos, true, Pathfinding::FREE | Pathfinding::WALL, front_ == BaseAction::FRONT) != Pathfinding::FREE)
                    {
                        if (nav_->GoToRotateFirst(pos, true, Pathfinding::FREE | Pathfinding::WALL | Pathfinding::OBSTACLE, front_ == BaseAction::FRONT) != Pathfinding::FREE)
                        {
                            status_ = MissionStatus::FAILED;
                            break;
                        }
                    }
                }

                go_timeout_ = node_->now();
            }
            break;
        case GO_TO_TAKE_CLOSE:
            {
                if (action_executor_->box_obstacles_[front_] == nullptr)
                {
                    status_ = MissionStatus::FAILED;
                    break;
                }

                auto pos = action_executor_->box_obstacles_[front_]->GetOptimizedGetPos(nav_->GetCurrentPos()).GetTakeClosePosition();
                pos.theta += front_ == BaseAction::FRONT ? 0 : M_PI;

                if (nav_->GoToRotateFirst(pos, true, Pathfinding::FREE | Pathfinding::WALL | Pathfinding::OBSTACLE, front_ == BaseAction::FRONT) != Pathfinding::FREE)
                {
                    status_ = MissionStatus::FAILED;
                    break;
                }

                go_timeout_ = node_->now();
            }
            break;
        case DOWN:
            {
                action_executor_->Down(front_);
                deploy_time_ = node_->now();
            }
            break;
        case TAKE:
            {
                action_executor_->Take({{0, front_}, {1, front_}, {2, front_}, {3, front_}});
                deploy_time_ = node_->now();
                min_time_ = node_->now() + rclcpp::Duration::from_seconds(0.5);
            }
            break;
        case UP:
            {
                action_executor_->Up(front_);
                deploy_time_ = node_->now();
            }
            break;
        case DONE:
            {
                if (action_executor_->box_obstacles_[front_] == nullptr)
                {
                    status_ = MissionStatus::FAILED;
                    break;
                }

                nav_->GetPathfinding()->RemoveObstacle(action_executor_->box_obstacles_[front_]->GetId());
            }

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
