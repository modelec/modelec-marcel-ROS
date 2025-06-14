#include <modelec_strat/missions/prepare_concert_mission.hpp>
#include <modelec_strat/obstacle/column.hpp>
#include <modelec_utils/config.hpp>

namespace Modelec
{
    PrepareConcertMission::PrepareConcertMission(const std::shared_ptr<NavigationHelper>& nav,
                                                 const std::shared_ptr<ActionExecutor>& action_executor,
                                                 bool two_floor) :
        step_(GO_TO_COLUMN),
        status_(MissionStatus::READY), nav_(nav), action_executor_(action_executor), two_floor_(two_floor)
    {
    }

    void PrepareConcertMission::Start(rclcpp::Node::SharedPtr node)
    {
        node_ = node;

        if (two_floor_)
        {
            mission_score_ = Config::get<int>("config.mission_score.concert.niv_2", 0);
        }
        else
        {
            mission_score_ = Config::get<int>("config.mission_score.concert.niv_1", 0);
        }

        score_pub_ = node_->create_publisher<std_msgs::msg::Int64>("/strat/score", 10);

        if (!nav_->GetClosestDepositeZone(nav_->GetCurrentPos(), nav_->GetTeamId()))
        {
            status_ = MissionStatus::FINISH_ALL;
            return;
        }

        int timeout = 0;
        std::vector<int> blacklistId = {};
        bool canGo = false;

        while (!canGo && timeout < 10)
        {
            if (auto col = nav_->GetPathfinding()->GetClosestColumn(nav_->GetCurrentPos(), blacklistId))
            {
                column_ = col;

                auto pos = column_->GetOptimizedGetPos(nav_->GetCurrentPos()).GetTakePosition(400);

                auto res = nav_->GoToRotateFirst(pos, false, Pathfinding::FREE | Pathfinding::WALL);
                if (res != Pathfinding::FREE)
                {
                    RCLCPP_WARN(node_->get_logger(), "Cannot go to column %d, trying another one", column_->GetId());
                    blacklistId.push_back(column_->GetId());
                }
                else
                {
                    canGo = true;
                }
            }

            timeout++;
        }

        if (!canGo)
        {
            status_ = MissionStatus::FINISH_ALL;
            return;
        }

        status_ = MissionStatus::RUNNING;
    }

    void PrepareConcertMission::Update()
    {
        if (status_ != MissionStatus::RUNNING)
        {
            return;
        }

        if (!nav_->HasArrived())
        {
            return;
        }

        if (!action_executor_->IsActionDone())
        {
            return;
        }

        switch (step_)
        {
        case GO_TO_COLUMN:
            {
                auto pos = column_->GetOptimizedGetPos(nav_->GetCurrentPos()).GetTakeClosePosition();
                nav_->GoTo(pos, true, Pathfinding::FREE | Pathfinding::WALL | Pathfinding::OBSTACLE);
            }

            step_ = GO_CLOSE_TO_COLUMN;
            break;
        case GO_CLOSE_TO_COLUMN:
            action_executor_->TakePot(two_floor_);
            nav_->GetPathfinding()->RemoveObstacle(column_->GetId());

            step_ = TAKE_COLUMN;
            break;
        case TAKE_COLUMN:
            {
                closestDepoZone_ = nav_->GetClosestDepositeZone(nav_->GetCurrentPos(), nav_->GetTeamId());
                if (!closestDepoZone_)
                {
                    status_ = MissionStatus::FAILED;
                    return;
                }

                closestDepoZonePoint_ = closestDepoZone_->GetNextPotPos();
                auto p = closestDepoZonePoint_.GetTakeBasePosition();
                auto res = nav_->CanGoTo(p, false, Pathfinding::FREE | Pathfinding::WALL);
                if (res != Pathfinding::FREE)
                {
                    auto pos = column_->GetOptimizedGetPos(nav_->GetCurrentPos()).GetTakeBasePosition();
                    nav_->GoTo(pos, true, Pathfinding::FREE | Pathfinding::WALL | Pathfinding::OBSTACLE);
                    step_ = GO_BACK;
                }
                else
                {
                    nav_->GoToRotateFirst(p, false, Pathfinding::FREE | Pathfinding::WALL);
                    closestDepoZone_->ValidNextPotPos();
                    step_ = GO_TO_PLATFORM;
                }
            }

            break;
        case GO_BACK:
            {
                bool canGo = false;
                std::vector<int> blacklistId = {};
                int timeout = 0;

                while (!canGo && timeout < 3)
                {
                    if (auto zone = nav_->GetClosestDepositeZone(nav_->GetCurrentPos(), nav_->GetTeamId(), blacklistId))
                    {
                        closestDepoZone_ = zone;
                        closestDepoZonePoint_ = closestDepoZone_->GetNextPotPos();
                        auto p = closestDepoZonePoint_.GetTakeBasePosition();

                        if (nav_->CanGoTo(p, false, Pathfinding::FREE | Pathfinding::WALL) != Pathfinding::FREE)
                        {
                            if (nav_->GoToRotateFirst(p, true, Pathfinding::FREE | Pathfinding::WALL) !=
                                Pathfinding::FREE)
                            {
                                blacklistId.push_back(closestDepoZone_->GetId());
                            }
                            else
                            {
                                canGo = true;
                            }
                        }
                        else
                        {
                            nav_->GoToRotateFirst(p, false, Pathfinding::FREE | Pathfinding::WALL);
                            canGo = true;
                        }
                    }
                    else
                    {
                        status_ = MissionStatus::FINISH_ALL;
                        return;
                    }

                    timeout++;
                }

                if (!canGo)
                {
                    status_ = MissionStatus::FINISH_ALL;
                    return;
                }

                closestDepoZonePoint_ = closestDepoZone_->ValidNextPotPos();
            }

            step_ = GO_TO_PLATFORM;
            break;
        case GO_TO_PLATFORM:
            {
                auto p = closestDepoZonePoint_.GetTakeClosePosition();
                nav_->GoTo(p.x, p.y, p.theta, true, Pathfinding::FREE | Pathfinding::WALL | Pathfinding::OBSTACLE);
            }

            step_ = GO_CLOSE_TO_PLATFORM;
            break;

        case GO_CLOSE_TO_PLATFORM:
            {
                action_executor_->PlacePot(two_floor_);

                column_->SetX(closestDepoZonePoint_.x);
                column_->SetY(closestDepoZonePoint_.y);
                column_->SetTheta(closestDepoZonePoint_.theta);
                column_->SetAtObjective(true);
                nav_->GetPathfinding()->AddObstacle(column_);
            }

            step_ = PLACE_PLATFORM;
            break;

        case PLACE_PLATFORM:
            {
                auto p = closestDepoZonePoint_.GetTakeBasePosition();
                if (nav_->CanGoTo(p.x, p.y, p.theta) != Pathfinding::FREE)
                {
                    nav_->GoTo(p.x, p.y, p.theta, true, Pathfinding::FREE | Pathfinding::WALL | Pathfinding::OBSTACLE);
                }
            }

            step_ = GO_BACK_2;
            break;

        case GO_BACK_2:
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

    void PrepareConcertMission::Clear()
    {
        // TODO : if is doing construction, stop everything and release everything
    }

    MissionStatus PrepareConcertMission::GetStatus() const
    {
        return status_;
    }
}
