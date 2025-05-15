#include <modelec_strat/missions/banner_mission.hpp>

#include <modelec_utils/config.hpp>

namespace Modelec
{
    BannerMission::BannerMission(const std::shared_ptr<NavigationHelper>& nav, const std::shared_ptr<ActionExecutor>& action_executor) : step_(GO_TO_FRONT),
        status_(MissionStatus::READY), nav_(nav), action_executor_(action_executor)
    {
    }

    void BannerMission::start(rclcpp::Node::SharedPtr node)
    {
        node_ = node;

        mission_score_ = Config::get<int>("config.mission_score.banner", 0);

        score_pub_ = node_->create_publisher<std_msgs::msg::Int64>("/strat/score", 10);

        auto spawn = nav_->GetSpawn();

        nav_->GoTo(spawn.x, (nav_->GetPathfinding()->robot_length_mm_ / 2) + 5, M_PI_2, true);

        status_ = MissionStatus::RUNNING;
    }

    void BannerMission::update()
    {
        if (status_ != MissionStatus::RUNNING) return;

        if (!nav_->HasArrived()) return;

        if (!action_executor_->IsActionDone()) return;

        switch (step_)
        {
        case GO_TO_FRONT:
            action_executor_->DeployBanner();

            step_ = DEPLOY_BANNER;
            break;

        case DEPLOY_BANNER:
            {
                auto spawn = nav_->GetSpawn();

                nav_->GoTo(spawn.x, (nav_->GetPathfinding()->robot_length_mm_ / 2) + 150, M_PI_2, true);

                step_ = GO_FORWARD;
                break;
            }
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

    void BannerMission::clear()
    {
    }

    MissionStatus BannerMission::getStatus() const
    {
        return status_;
    }
}
