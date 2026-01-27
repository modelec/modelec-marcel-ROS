#include <modelec_strat/missions/free_mission.hpp>

#include "modelec_strat/action/base_action.hpp"

namespace Modelec {
    FreeMission::FreeMission(const std::shared_ptr<NavigationHelper>& nav,
        const std::shared_ptr<ActionExecutor>& action_executor,
        BaseAction::Front front)
     : front_(front), status_(MissionStatus::READY), nav_(nav), action_executor_(action_executor)
    {
    }

    void FreeMission::Start(rclcpp::Node::SharedPtr node)
    {
        node_ = node;

        go_timeout_ = node_->now();
        deploy_time_ = node_->now();
        last_ask_waypoint_time_ = node_->now();

        status_ = MissionStatus::RUNNING;

        std::queue<int> empty;
        std::swap(steps_, empty);

        steps_.push(GO_TO_FREE);
        steps_.push(DOWN);
        steps_.push(FREE);
        steps_.push(UP);
        steps_.push(GO_BACK);
        steps_.push(DONE);
    }

    void FreeMission::Update()
    {
        if (!action_executor_->IsActionDone())
        {
            return;
        }

        if (!nav_->HasArrived())
        {
            if ((node_->now() - go_timeout_).seconds() < 2 && (node_->now() - last_ask_waypoint_time_).seconds() > 2)
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
        case GO_TO_FREE:
            {
                auto currPos = nav_->GetCurrentPos();

                auto dist = std::clamp(Point::distance(Point(currPos->x, currPos->y, currPos->theta),
                    nav_->GetClosestDepositeZone(nav_->GetCurrentPos())->GetPosition()), 0.0, 200.0);

                target_deposite_zone_ = nav_->GetClosestDepositeZone(nav_->GetCurrentPos(), {}, true);

                auto depoPoint = target_deposite_zone_->GetBestTakePosition(Point(currPos->x, currPos->y, currPos->theta));

                auto pos = depoPoint.GetTakePosition(dist);

                RCLCPP_INFO(
                    node_->get_logger(),
                    "FreeMission: position (%.2d, %.2d) with distance %.2f",
                    pos.x, pos.y, dist);

                pos.theta += front_ == BaseAction::FRONT ? 0 : M_PI;

                if (nav_->GoToRotateFirst(pos, true, Pathfinding::FREE, front_ == BaseAction::FRONT) != Pathfinding::FREE)
                {
                    if (nav_->GoToRotateFirst(pos, true, Pathfinding::FREE | Pathfinding::OBSTACLE, front_ == BaseAction::FRONT) != Pathfinding::FREE)
                    {
                        status_ = MissionStatus::FAILED;
                        return;
                    }
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
        case FREE:
            {
                action_executor_->Free({{0, front_}, {1, front_}, {2, front_}, {3, front_}});
                deploy_time_ = node_->now();

                auto obs = action_executor_->box_obstacles_[front_];
                action_executor_->box_obstacles_[front_] = nullptr;

                auto pos = nav_->GetCurrentPos();

                obs->SetPosition(
                    pos->x + 200 * cos(pos->theta),
                    pos->y + 200 * sin(pos->theta),
                    pos->theta);

                obs->SetAtObjective(true);

                nav_->GetPathfinding()->AddObstacle(obs);

                min_time_ = node_->now() + rclcpp::Duration::from_seconds(0.5);
            }
            break;
        case UP:
            {
                action_executor_->Up(front_);
                deploy_time_ = node_->now();
            }
            break;
        case GO_BACK:
            {
                auto currPos = nav_->GetCurrentPos();

                auto depoPoint = target_deposite_zone_->GetBestTakePosition(Point(currPos->x, currPos->y, currPos->theta));

                auto pos = depoPoint.GetTakePosition(300);
                pos.theta += front_ == BaseAction::FRONT ? 0 : M_PI;

                if (nav_->GoTo(pos, true, Pathfinding::FREE | Pathfinding::OBSTACLE) != Pathfinding::FREE)
                {
                    status_ = MissionStatus::FAILED;
                    return;
                }

                go_timeout_ = node_->now();
            }
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
