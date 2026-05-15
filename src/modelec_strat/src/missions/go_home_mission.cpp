#include <modelec_strat/missions/go_home_mission.hpp>

#include <modelec_utils/config.hpp>

namespace Modelec
{
    GoHomeMission::GoHomeMission(const std::shared_ptr<NavigationHelper>& nav, const std::shared_ptr<ActionExecutor>& action_executor, const rclcpp::Time& start_time) :
        Mission(MissionStatus::READY), MoveMission(nav), MinTimeMission(), ActionMission(action_executor),
        start_time_(start_time)
    {
    }

    void GoHomeMission::Start(const rclcpp::Node::SharedPtr& node)
    {
        MoveMission::Start(node);
        MinTimeMission::Start(node);
        ActionMission::Start(node);

        mission_score_ = Config::get<int>("config.mission_score.go_home", 0);

        score_pub_ = node_->create_publisher<std_msgs::msg::Int64>("/strat/score", 10);

        status_ = MissionStatus::RUNNING;

        std::queue<int> empty;
        std::swap(steps_, empty);

        steps_.push(ROTATE_TO_HOME);
        steps_.push(AWAIT_90);
        steps_.push(GO_HOME);
        steps_.push(GO_CLOSE);
        steps_.push(RELEASE_BLOCK_IF_NOT_EMPTY);
        steps_.push(DONE);
    }

    bool GoHomeMission::Update()
    {
        if (!MoveMission::Update() || !MinTimeMission::Update() || !ActionMission::Update())
        {
            return false;
        }

        auto step_ = steps_.front();
        steps_.pop();

        switch (step_)
        {
        case ROTATE_TO_HOME:
            {
                auto pos = nav_->GetHomePosition();
                home_point_ = Point(pos->x, pos->y, pos->theta);
                if (nav_->CanGoTo(home_point_.GetTakeBasePosition()) != Pathfinding::FREE)
                {
                    if (nav_->CanGoTo(home_point_.GetTakeBasePosition(), true) != Pathfinding::FREE)
                    {
                        status_ = MissionStatus::FAILED;
                        return false;
                    }
                }
                nav_->RotateTo(home_point_);

                go_timeout_ = node_->now();
            }
            break;
        case AWAIT_90:
            {
                min_time_ = start_time_ + rclcpp::Duration(90, 0);
            }

            break;
        case GO_HOME:
            {
                if (nav_->GoTo(home_point_.GetTakeBasePosition()) != Pathfinding::FREE)
                {
                    if (nav_->GoTo(home_point_.GetTakeBasePosition(), true) != Pathfinding::FREE)
                    {
                        status_ = MissionStatus::FAILED;
                        return false;
                    }
                }

                go_timeout_ = node_->now();
            }
            break;
        case GO_CLOSE:
            {
                nav_->GoTo(home_point_, true);

                go_timeout_ = node_->now();
            }
            break;
        case RELEASE_BLOCK_IF_NOT_EMPTY:
            {
                if (action_executor_->HasOneBox())
                {
                    action_executor_->Down(BaseAction::BOTH);
                    action_executor_->Free({{0, BaseAction::FRONT}, {1, BaseAction::FRONT}, {2, BaseAction::FRONT}, {3, BaseAction::FRONT},
                        {0, BaseAction::BACK}, {1, BaseAction::BACK}, {2, BaseAction::BACK}, {3, BaseAction::BACK}});
                    deploy_time_ = node_->now();
                }
            }

            break;
        case DONE:
            {
                std_msgs::msg::Int64 score_msg;
                score_msg.data = mission_score_;
                score_pub_->publish(score_msg);

                status_ = MissionStatus::DONE;
            }
            break;
        default:
            break;
        }

        return true;
    }

    void GoHomeMission::Clear()
    {
    }

}
