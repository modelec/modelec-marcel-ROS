#include <modelec_strat/missions/banner_mission.hpp>

#include <modelec_utils/config.hpp>

namespace Modelec
{
    BannerMission::BannerMission(const std::shared_ptr<NavigationHelper>& nav,
                                 const std::shared_ptr<ActionExecutor>& action_executor) : step_(GO_TO_FRONT),
        status_(MissionStatus::READY), nav_(nav), action_executor_(action_executor)
    {
    }

    void BannerMission::Start(rclcpp::Node::SharedPtr node)
    {
        node_ = node;

        mission_score_ = Config::get<int>("config.mission_score.banner", 0);

        score_pub_ = node_->create_publisher<std_msgs::msg::Int64>("/strat/score", 10);

        spawn_ = nav_->GetSpawn();

        nav_->GoTo(spawn_.x, (nav_->GetPathfinding()->robot_length_mm_ / 2) + 5, M_PI_2, true, Pathfinding::FREE | Pathfinding::WALL | Pathfinding::OBSTACLE);

        status_ = MissionStatus::RUNNING;
    }

    void BannerMission::Update()
    {
        if (status_ != MissionStatus::RUNNING)
        {
            return;
        }

        if (!action_executor_->IsActionDone())
        {
            return;
        }

        if (!nav_->HasArrived())
        {
            return;
        }

        switch (step_)
        {
        case GO_TO_FRONT:
            {
                action_executor_->DeployBanner();
                deploy_time_ = node_->now();

                step_ = DEPLOY_BANNER;
            }

            break;

        case DEPLOY_BANNER:
            {
                if ((node_->now() - deploy_time_).seconds() >= 5)
                {
                    step_ = WAIT_5_SECONDS;
                }
            }

            break;
        case WAIT_5_SECONDS:
            {
                action_executor_->UndeployBanner();

                step_ = UNDEPLOY_BANNER;
            }

            break;

        case UNDEPLOY_BANNER:
            {
                nav_->GoTo(spawn_.x, (nav_->GetPathfinding()->robot_length_mm_ / 2) + 600, M_PI_2, true, Pathfinding::FREE | Pathfinding::WALL | Pathfinding::OBSTACLE);

                step_ = GO_FORWARD;
            }

            break;

        case GO_FORWARD:
            {
                std_msgs::msg::Int64 msg;
                msg.data = mission_score_;
                score_pub_->publish(msg);

                step_ = DONE;
                status_ = MissionStatus::DONE;
                break;
            }

        default:
            break;
        }
    }

    void BannerMission::Clear()
    {
    }

    MissionStatus BannerMission::GetStatus() const
    {
        return status_;
    }
}
