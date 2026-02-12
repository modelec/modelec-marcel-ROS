#include <modelec_strat/missions/take_mission.hpp>

#include "modelec_strat/action/base_action.hpp"

namespace Modelec {
    TakeMission::TakeMission(const std::shared_ptr<NavigationHelper>& nav,
        const std::shared_ptr<ActionExecutor>& action_executor, BaseAction::Side side) :
    Mission(MissionStatus::READY), ActionMission(action_executor), MoveMission(nav),
    side_(side)
    {
    }

    void TakeMission::Start(rclcpp::Node::SharedPtr node)
    {
        ActionMission::Start(node);
        MoveMission::Start(node);
        MinTimeMission::Start(node);

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

    bool TakeMission::Update()
    {
        if (!ActionMission::Update() || !MoveMission::Update() || !MinTimeMission::Update())
        {
            return false;
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

                action_executor_->box_obstacles_[side_] = closestBox;

                auto pos = closestBox->GetOptimizedGetPos(nav_->GetCurrentPos()).GetTakeBasePosition();
                pos.theta = Point::normalizeAngle(pos.theta + (side_ == BaseAction::FRONT ? 0 : M_PI));

                if (nav_->GoToRotateFirst(pos, true, Pathfinding::FREE, side_ == BaseAction::FRONT) != Pathfinding::FREE)
                {
                    if (nav_->GoToRotateFirst(pos, true, Pathfinding::FREE | Pathfinding::OBSTACLE, side_ == BaseAction::FRONT) != Pathfinding::FREE)
                    {
                        status_ = MissionStatus::FAILED;
                        return false;
                    }
                }

                go_timeout_ = node_->now();
            }
            break;
        case GO_TO_TAKE_CLOSE:
            {
                if (action_executor_->box_obstacles_[side_] == nullptr)
                {
                    status_ = MissionStatus::FAILED;
                    return false;
                }

                auto pos = action_executor_->box_obstacles_[side_]->GetOptimizedGetPos(nav_->GetCurrentPos()).GetTakeClosePosition();
                pos.theta = Point::normalizeAngle(pos.theta + (side_ == BaseAction::FRONT ? 0 : M_PI));

                if (nav_->GoToRotateFirst(pos, true, Pathfinding::FREE | Pathfinding::WALL | Pathfinding::OBSTACLE, side_ == BaseAction::FRONT) != Pathfinding::FREE)
                {
                    status_ = MissionStatus::FAILED;
                    return false;
                }

                go_timeout_ = node_->now();
            }
            break;
        case DOWN:
            {
                action_executor_->RotateArm(side_, false, false);

                deploy_time_ = node_->now();
            }
            break;
        case TAKE:
            {
                action_executor_->Take({{0, side_}, {1, side_}, {2, side_}, {3, side_}});
                deploy_time_ = node_->now();
                min_time_ = node_->now() + rclcpp::Duration::from_seconds(0.5);
            }
            break;
        case UP:
            {
                action_executor_->Up(side_);
                action_executor_->LookOn(side_);
                deploy_time_ = node_->now();
            }
            break;
        case DONE:
            {
                if (action_executor_->box_obstacles_[side_] == nullptr)
                {
                    status_ = MissionStatus::FAILED;
                    return false;
                }

                nav_->GetPathfinding()->RemoveObstacle(action_executor_->box_obstacles_[side_]->GetId());

                action_executor_->AskColor();
            }

            status_ = MissionStatus::DONE;
            break;
        default:
            break;
        }

        return true;
    }

    void TakeMission::Clear()
    {
    }

}
