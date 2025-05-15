#include <modelec_strat/missions/go_home_mission.hpp>

#include <modelec_utils/config.hpp>

namespace Modelec
{
    GoHomeMission::GoHomeMission(const std::shared_ptr<NavigationHelper>& nav, const rclcpp::Time& start_time) :
        step_(ROTATE_TO_HOME), status_(MissionStatus::READY), nav_(nav), start_time_(start_time)
    {
    }

    void GoHomeMission::Start(rclcpp::Node::SharedPtr node)
    {
        node_ = node;

        mission_score_ = Config::get<int>("config.mission_score.go_home", 0);

        score_pub_ = node_->create_publisher<std_msgs::msg::Int64>("/strat/score", 10);

        auto pos = nav_->GetHomePosition();
        home_point_ = Point(pos->x, pos->y, pos->theta);
        if (nav_->CanGoTo(home_point_.GetTakeBasePosition()) != Pathfinding::FREE)
        {
            if (nav_->CanGoTo(home_point_.GetTakeBasePosition(), true) != Pathfinding::FREE)
            {
                status_ = MissionStatus::FAILED;
                return;
            }
        }
        nav_->RotateTo(home_point_);

        status_ = MissionStatus::RUNNING;
    }

    void GoHomeMission::Update()
    {
        if (!nav_->HasArrived()) return;

        switch (step_)
        {
        case ROTATE_TO_HOME:
            {
                if (nav_->GoTo(home_point_.GetTakeBasePosition()) != Pathfinding::FREE)
                {
                    if (nav_->GoTo(home_point_.GetTakeBasePosition(), true) != Pathfinding::FREE)
                    {
                        status_ = MissionStatus::FAILED;
                        return;
                    }
                }
            }

            step_ = GO_HOME;
            break;
        case GO_HOME:
            if ((node_->now() - start_time_).seconds() < 94)
            {
                break;
            }

            nav_->GoTo(home_point_.GetTakePosition(0), true);

            step_ = GO_CLOSE;
            break;

        case GO_CLOSE:
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

    void GoHomeMission::Clear()
    {
    }

    MissionStatus GoHomeMission::GetStatus() const
    {
        return status_;
    }
}
