#include <modelec_strat/strat_fms.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <modelec_utils/config.hpp>

#include <modelec_strat/missions/go_home_mission.hpp>
#include <modelec_strat/missions/take_mission.hpp>
#include <modelec_strat/missions/free_mission.hpp>

#include "modelec_strat/action/base_action.hpp"

namespace Modelec
{

    StratFMS::StratFMS() : Node("start_fms")
    {
        tir_sub_ = create_subscription<std_msgs::msg::Empty>(
            "/action/tir/start", 10, [this](const std_msgs::msg::Empty::SharedPtr)
            {
                if (!started_)
                {
                    started_ = true;
                }
            });

        state_pub_ = create_publisher<modelec_interfaces::msg::StratState>("/strat/state", 10);

        start_time_pub_ = create_publisher<std_msgs::msg::Int64>("/strat/start_time", 10);

        team_id_sub_ = create_subscription<std_msgs::msg::Int8>(
            "/strat/team", 10, [this](const std_msgs::msg::Int8::SharedPtr msg)
            {
                team_id_ = static_cast<int>(msg->data);
                nav_->SetTeamId(team_id_);
            });

        spawn_id_sub_ = create_subscription<modelec_interfaces::msg::Spawn>(
            "/strat/spawn", 10, [this](const modelec_interfaces::msg::Spawn::SharedPtr msg)
            {
                team_selected_ = true;
                team_id_ = msg->team_id;
                nav_->SetTeamId(team_id_);
                nav_->SetSpawn(msg->name);
            });

        reset_strat_sub_ = create_subscription<std_msgs::msg::Empty>(
            "/strat/reset", 10, [this](const std_msgs::msg::Empty::SharedPtr)
            {
                Reset();
            });

        tir_arm_sub_ = create_subscription<std_msgs::msg::Empty>(
            "/action/tir/arm", 10, [this](const std_msgs::msg::Empty::SharedPtr)
            {
                setup_ = true;
            });

        tir_arm_set_pub_ = create_publisher<std_msgs::msg::Bool>(
            "/action/tir/arm/set", 10);

        start_odo_pub_ = create_publisher<std_msgs::msg::Bool>("/odometry/start", 10);

        std::string config_path = ament_index_cpp::get_package_share_directory("modelec_strat") + "/data/config.xml";
        if (!Config::load(config_path))
        {
            RCLCPP_ERROR(get_logger(), "Failed to load config file: %s", config_path.c_str());
        }

        game_action_sequence_.push(State::TAKE_MISSION);
        game_action_sequence_.push(State::FREE_MISSION);
        game_action_sequence_.push(State::TAKE_MISSION);
        game_action_sequence_.push(State::FREE_MISSION);
    }

    void StratFMS::Init()
    {
        nav_ = std::make_shared<NavigationHelper>(shared_from_this());
        action_executor_ = std::make_unique<ActionExecutor>(shared_from_this());
        ResetStrat();

        RCLCPP_INFO(this->get_logger(), "StratFMS fully initialized");
    }

    void StratFMS::ReInit()
    {
        nav_->ReInit();
        action_executor_->ReInit();
        setup_ = false;
    }

    void StratFMS::Reset()
    {
        ReInit();
        ResetStrat();
    }

    void StratFMS::ResetStrat()
    {
        if (timer_)
        {
            timer_->cancel();
        }

        state_ = State::INIT;
        started_ = false;
        team_selected_ = false;
        current_mission_.reset();
        match_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

        timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]
        {
            Update();
        });
    }

    void StratFMS::Transition(State next, const std::string& reason)
    {
        RCLCPP_INFO(get_logger(), "Transition %d -> %d: %s", static_cast<int>(state_), static_cast<int>(next),
                    reason.c_str());
        state_ = next;
        modelec_interfaces::msg::StratState msg;
        msg.state = static_cast<int>(state_);
        msg.reason = reason;
        state_pub_->publish(msg);
    }

    void StratFMS::Update()
    {
        auto now = this->now();

        switch (state_)
        {
        case State::INIT:
            if (team_selected_)
            {
                started_ = false;

                std_msgs::msg::Bool start_odo_msg;
                start_odo_msg.data = true;
                start_odo_pub_->publish(start_odo_msg);

                std_msgs::msg::Bool arm_msg;
                arm_msg.data = true;
                tir_arm_set_pub_->publish(arm_msg);

                action_executor_->Up(BaseAction::Front::BOTH, true);
                action_executor_->Free({
                    {0, BaseAction::FRONT}, {1, BaseAction::FRONT}, {2, BaseAction::FRONT}, {3, BaseAction::FRONT},
                    {0, BaseAction::BACK}, {1, BaseAction::BACK}, {2, BaseAction::BACK}, {3, BaseAction::BACK},
                }, true);

                Transition(State::WAIT_START, "System ready");
            }
            break;
        case State::WAIT_START:
            if (started_)
            {
                rclcpp::sleep_for(std::chrono::milliseconds(300));

                match_start_time_ = now;

                std_msgs::msg::Int64 msg;
                msg.data = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::system_clock::now().time_since_epoch())
                               .count();
                start_time_pub_->publish(msg);

                Transition(State::SELECT_MISSION, "Match started");
            }
            break;

        case State::SELECT_MISSION:
            {
                auto elapsed = now - match_start_time_;

                if (elapsed.seconds() >= 100)
                {
                    Transition(State::STOP, "All missions done");
                }

                else if (elapsed.seconds() < 70)
                {
                    Transition(State::SELECT_GAME_ACTION, "Selecting a game action");
                }
                else
                {
                    Transition(State::DO_GO_HOME, "Cleanup and finish match");
                }

                break;
            }
        case State::SELECT_GAME_ACTION:
            {
                if (action_executor_->IsEmpty())
                {
                    RCLCPP_INFO(get_logger(), "Missing box on robot, selecting TAKE mission");
                    Transition(State::TAKE_MISSION, "Selecting TAKE mission");
                }
                else
                {
                    RCLCPP_INFO(get_logger(), "All box are present on robot, selecting FREE mission");
                    Transition(State::FREE_MISSION, "Selecting FREE mission");
                }
            }

            break;
        case State::TAKE_MISSION:
            if (!current_mission_)
            {
                if (action_executor_->HasFrontBox())
                {
                    if (action_executor_->HasBackBox())
                    {
                        RCLCPP_WARN(get_logger(), "Both front and back box obstacles are occupied!");
                        current_mission_.reset();
                        Transition(State::SELECT_MISSION, "Cannot take box, both sides occupied");
                        break;
                    }

                    RCLCPP_INFO(get_logger(), "Front box obstacle is occupied, taking from back");
                    current_mission_ = std::make_unique<TakeMission>(nav_, action_executor_, BaseAction::BACK);
                    current_mission_->Start(shared_from_this());
                }
                else
                {
                    current_mission_ = std::make_unique<TakeMission>(nav_, action_executor_, BaseAction::FRONT);
                    current_mission_->Start(shared_from_this());
                }
            }
            current_mission_->Update();
            if (current_mission_->GetStatus() == MissionStatus::DONE)
            {
                current_mission_.reset();
                Transition(State::SELECT_MISSION, "Take done");
            }
            break;

        case State::FREE_MISSION:
            if (!current_mission_)
            {
                if (!action_executor_->HasFrontBox())
                {
                    if (!action_executor_->HasBackBox())
                    {
                        RCLCPP_WARN(get_logger(), "Both front and back box obstacles are free!");
                        Transition(State::SELECT_MISSION, "Cannot free box, both sides empty");
                        break;
                    }

                    RCLCPP_INFO(get_logger(), "Front box obstacle is occupied, taking from back");
                    current_mission_ = std::make_unique<FreeMission>(nav_, action_executor_, BaseAction::BACK);
                    current_mission_->Start(shared_from_this());
                }
                else
                {
                    current_mission_ = std::make_unique<FreeMission>(nav_, action_executor_, BaseAction::FRONT);
                    current_mission_->Start(shared_from_this());
                }
            }
            current_mission_->Update();
            if (current_mission_->GetStatus() == MissionStatus::DONE)
            {
                current_mission_.reset();
                Transition(State::SELECT_MISSION, "Free done");
            }
            break;

        case State::DO_GO_HOME:
            if (!current_mission_)
            {
                current_mission_ = std::make_unique<GoHomeMission>(nav_, match_start_time_);
                current_mission_->Start(shared_from_this());
            }
            current_mission_->Update();
            if (current_mission_->GetStatus() == MissionStatus::DONE)
            {
                current_mission_.reset();
                Transition(State::STOP, "Cleanup done");
            }
            break;

        case State::STOP:
            RCLCPP_INFO_ONCE(get_logger(), "State: STOP - Match finished");
            break;
        }
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Modelec::StratFMS>();
    node->Init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
