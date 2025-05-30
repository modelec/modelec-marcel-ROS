#include <modelec_strat/missions/go_home_mission.hpp>

#include <modelec_utils/config.hpp>

namespace Modelec
{
    GoHomeMission::GoHomeMission(const std::shared_ptr<NavigationHelper>& nav, const rclcpp::Time& start_time) :
        step_(GO_FRONT), status_(MissionStatus::READY), nav_(nav), start_time_(start_time)
    {
    }

    void GoHomeMission::Start(rclcpp::Node::SharedPtr node)
    {
        node_ = node;

        mission_score_ = Config::get<int>("config.mission_score.go_home", 0);

        score_pub_ = node_->create_publisher<std_msgs::msg::Int64>("/strat/score", 10);

        auto curr = nav_->GetCurrentPos();
        RCLCPP_INFO(node_->get_logger(), "GoHomeMission: Starting at position (%f, %f)", curr->x, curr->y);

        nav_->GoTo(375, 700, -M_PI_2, true, Pathfinding::FREE | Pathfinding::WALL | Pathfinding::OBSTACLE);

        go_home_time_ = node_->now();

        status_ = MissionStatus::RUNNING;
    }

    void GoHomeMission::Update()
    {
        if (!nav_->HasArrived())
        {
            return;
        }

        switch (step_)
        {
        case GO_FRONT:
            {
                if ((node_->now() - go_home_time_).seconds() >= 10)
                {
                    step_ = AWAIT_10S;
                }
            }

            break;

        case AWAIT_10S:
            {
                nav_->GoTo(375, 1500, -M_PI_2, true, Pathfinding::FREE | Pathfinding::WALL | Pathfinding::OBSTACLE);

                step_ = GO_HOME;
            }

            break;
        case GO_HOME:
            {
                std_msgs::msg::Int64 msg;
                msg.data = mission_score_;
                score_pub_->publish(msg);

                step_ = DONE;
                status_ = MissionStatus::DONE;
            }
            break;

        default:
            break;
        }
    }

    void GoHomeMission::Clear()
    {
    }

    MissionStatus GoHomeMission::GetStatus() const
    {
        return status_;
    }
}
