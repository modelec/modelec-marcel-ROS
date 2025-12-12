#include <modelec_strat/missions/free_mission.hpp>

namespace Modelec {
    FreeMission::FreeMission(const std::shared_ptr<NavigationHelper>& nav, const std::shared_ptr<ActionExecutor>& action_executor)
     : step_(GO_TO_FREE), status_(MissionStatus::READY), nav_(nav), action_executor_(action_executor)  {
    }

    void FreeMission::Start(rclcpp::Node::SharedPtr node)
    {
        node_ = node;

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
                    nav_->GetClosestDepositeZone(nav_->GetCurrentPos())->GetPosition()), 0.0, 200.0);

                auto depoPoint = nav_->GetClosestDepositeZone(nav_->GetCurrentPos())->GetPosition().GetTakePosition(dist,
                    nav_->GetCurrentPos()->theta);

                nav_->GoToRotateFirst(depoPoint, true, Pathfinding::FREE);

                go_timeout_ = node_->now();

                step_ = FREE;
            }
            break;
        case FREE:
            {
                action_executor_->Down();
                deploy_time_ = node_->now();

                step_ = DONE;
            }
            break;
        case DONE:
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