#include <modelec_strat/missions/free_mission.hpp>

#include "modelec_strat/action/base_action.hpp"

namespace Modelec {
    FreeMission::FreeMission(const std::shared_ptr<NavigationHelper>& nav,
        const std::shared_ptr<ActionExecutor>& action_executor, BaseAction::Side side) :
        Mission(MissionStatus::READY), ActionMission(action_executor), MoveMission(nav),
        side_(side)
    {
    }

    void FreeMission::Start(const rclcpp::Node::SharedPtr& node)
    {
        ActionMission::Start(node);
        MoveMission::Start(node);

        status_ = MissionStatus::RUNNING;

        std::queue<int> empty;
        std::swap(steps_, empty);

        steps_.push(GO_TO_FREE);
        steps_.push(CHECK_BOX);
        steps_.push(DOWN);
        steps_.push(FREE_FIRST);
        steps_.push(ROTATE_ARM);
        steps_.push(FREE_OTHER);
        steps_.push(UP);
        steps_.push(GO_BACK);
        steps_.push(DONE);
    }

    bool FreeMission::Update()
    {
        if (!ActionMission::Update() || !MoveMission::Update())
        {
            return false;
        }

		auto step_ = steps_.front();
		steps_.pop();

        switch (step_)
        {
        case GO_TO_FREE:
            {
                auto currPos = nav_->GetCurrentPos();

                target_deposite_zone_ = nav_->GetClosestDepositeZone(currPos, {}, true);

                if (target_deposite_zone_ == nullptr)
                {
                    status_ = MissionStatus::FAILED;
                    return false;
                }

                auto depoPoint = target_deposite_zone_->GetBestTakePosition(Point(currPos->x, currPos->y, currPos->theta));

                auto pos = depoPoint.GetTakePosition(200.0);

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
        case CHECK_BOX:
            {
                auto obs = action_executor_->box_obstacles_[side_];

                auto vect = obs->GetSide(nav_->GetTeamId() == NavigationHelper::BLUE ? BoxObstacle::BLUE : BoxObstacle::YELLOW);

                RCLCPP_DEBUG(node_->get_logger(), "Box on side %d has %d boxes of his color side", static_cast<int>(side_), static_cast<int>(vect.size()));

                if (vect.size() == 4)
                {
                    std::queue<int> empty;
                    std::swap(steps_, empty);

                    steps_.push(DOWN);
                    steps_.push(FREE_FIRST);
                    steps_.push(UP);
                    steps_.push(GO_BACK);
                    steps_.push(DONE);
                } else if (vect.empty())
                {
                    std::queue<int> empty;
                    std::swap(steps_, empty);

                    steps_.push(ROTATE_ARM);
                    steps_.push(FREE_OTHER);
                    steps_.push(UP);
                    steps_.push(GO_BACK);
                    steps_.push(DONE);
                }
            }

            break;
        case DOWN:
            {
                action_executor_->Down(side_);
                deploy_time_ = node_->now();
            }

            break;
        case FREE_FIRST:
            {
                auto obs = action_executor_->box_obstacles_[side_];

                auto vect = obs->GetSide(nav_->GetTeamId() == NavigationHelper::BLUE ? BoxObstacle::BLUE : BoxObstacle::YELLOW);

                auto servo = std::vector<std::pair<int, BaseAction::Side>>();
                for (auto s : vect)
                {
                    servo.push_back({s, side_});
                }

                action_executor_->Free(servo);

                deploy_time_ = node_->now();
            }
            break;
        case ROTATE_ARM:
            {
                action_executor_->RotateArm(side_, false, true);
                deploy_time_ = node_->now();
            }
            break;
        case FREE_OTHER:
            {
                action_executor_->Free({{0, side_}, {1, side_}, {2, side_}, {3, side_}});
                deploy_time_ = node_->now();
            }
            break;
        case UP:
            {
                action_executor_->Up(side_);
                deploy_time_ = node_->now();
            }
            break;
        case GO_BACK:
            {
                auto currPos = nav_->GetCurrentPos();

                auto depoPoint = target_deposite_zone_->GetBestTakePosition(Point(currPos->x, currPos->y, currPos->theta));

                auto pos = depoPoint.GetTakePosition(300);
                pos.theta = Point::normalizeAngle(pos.theta + (side_ == BaseAction::FRONT ? 0 : M_PI));

                if (nav_->GoTo(pos, true, Pathfinding::FREE | Pathfinding::OBSTACLE) != Pathfinding::FREE)
                {
                    status_ = MissionStatus::FAILED;
                    return false;
                }

                go_timeout_ = node_->now();
            }

            break;
        case DONE:
            {
                auto obs = action_executor_->box_obstacles_[side_];
                action_executor_->box_obstacles_[side_] = nullptr;

                auto pos = nav_->GetCurrentPos();

                obs->SetPosition(
                    target_deposite_zone_->GetPosition().x,
                    target_deposite_zone_->GetPosition().y,
                    pos->theta);

                obs->SetAtObjective(true);

                nav_->GetPathfinding()->AddObstacle(obs);

                target_deposite_zone_->Validate(true);
            }

            status_ = MissionStatus::DONE;
            break;
        default:
            break;
        }

        return true;
    }

    void FreeMission::Clear()
    {
    }

}
