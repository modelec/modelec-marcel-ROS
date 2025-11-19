#include <modelec_strat/missions/take_send_mission.hpp>

namespace Modelec {
    TakeSendMission::TakeSendMission(const std::shared_ptr<NavigationHelper>& nav, const std::shared_ptr<ActionExecutor>& action_executor)
     : step_(GO_TO_TAKE), status_(MissionStatus::READY), nav_(nav), action_executor_(action_executor)  {
    }

    void TakeSendMission::Start(rclcpp::Node::SharedPtr node)
    {
        node_ = node;


        nav_->GoTo/*RotateFirst*/(2500, 1200, M_PI_2, true, Pathfinding::FREE | Pathfinding::WALL | Pathfinding::OBSTACLE);

        go_timeout_ = node_->now();

        status_ = MissionStatus::RUNNING;
    }

    void TakeSendMission::Update()
    {
        if (!action_executor_->IsActionDone())
        {
            return;
        }

        if (!nav_->HasArrived())
        {
            if ((node_->now() - go_timeout_).seconds() < 4)
            {
                return;
            }
        }

        switch (step_)
        {
        case GO_TO_TAKE:
            action_executor_->Take();
            deploy_time_ = node_->now();

            step_ = TAKE;
            break;
        case TAKE:
            if ((node_->now() - deploy_time_).seconds() >= 5)
            {
                step_ = WAIT_5S;
            }

            break;
        case WAIT_5S:
            nav_->GoTo/*RotateFirst*/(2000, 700, 0, true, Pathfinding::FREE | Pathfinding::WALL | Pathfinding::OBSTACLE);

            step_ = GO_TO_SEND;
            break;
        case GO_TO_SEND:
            action_executor_->Send();
            deploy_time_ = node_->now();

            step_ = SEND;
            break;
        case SEND:

            step_ = DONE;
            status_ = MissionStatus::DONE;
            break;
        default:
            break;
        }
    }

    void TakeSendMission::Clear()
    {
    }

    MissionStatus TakeSendMission::GetStatus() const
    {
        return status_;
    }

}