#include <ament_index_cpp/get_package_share_directory.hpp>
#include <modelec_utils/config.hpp>
#include <modelec_strat/strat_fms.hpp>

namespace Modelec
{

    StratFMS::StratFMS() : Node("start_fms")
    {
        tir_sub_ = create_subscription<std_msgs::msg::Empty>(
            "/action/tir/start", 10, [this](const std_msgs::msg::Empty::SharedPtr)
            {
                if (setup_ && !started_)
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

        start_odo_pub_ = create_publisher<std_msgs::msg::Bool>("/odometry/start", 10);

        std::string config_path = ament_index_cpp::get_package_share_directory("modelec_strat") + "/data/config.xml";
        if (!Config::load(config_path))
        {
            RCLCPP_ERROR(get_logger(), "Failed to load config file: %s", config_path.c_str());
        }
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
            if (setup_ && team_selected_)
            {
                std_msgs::msg::Bool start_odo_msg;
                start_odo_msg.data = true;
                start_odo_pub_->publish(start_odo_msg);

                Transition(State::WAIT_START, "System ready");
            }
            break;
        case State::WAIT_START:
            if (started_)
            {
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

                // TODO : next thing to upgrade to have a good strat
                if (!is_banner_done_)
                {
                    Transition(State::DO_PROMOTION, "Start promotion");
                }
                // TODO : check the time needed by the mission
                else if (elapsed.seconds() < 75)
                {
                    Transition(State::DO_PREPARE_CONCERT, "Proceed to concert");
                }
                else
                {
                    Transition(State::DO_GO_HOME, "Cleanup and finish match");
                }
                break;
            }

        case State::DO_PREPARE_CONCERT:
            if (!current_mission_)
            {
                current_mission_ = std::make_unique<PrepareConcertMission>(nav_, action_executor_, (now - match_start_time_).seconds() < 70);
                current_mission_->Start(shared_from_this());
            }
            current_mission_->Update();
            if (current_mission_->GetStatus() == MissionStatus::DONE)
            {
                current_mission_.reset();
                Transition(State::SELECT_MISSION, "PrepareConcert finished");
            }
            else if (current_mission_->GetStatus() == MissionStatus::FAILED)
            {
                current_mission_->Clear();
                current_mission_.reset();
                Transition(State::SELECT_MISSION, "PrepareConcert failed");
            }
            else if (current_mission_->GetStatus() == MissionStatus::FINISH_ALL)
            {
                current_mission_.reset();
                Transition(State::DO_GO_HOME, "Finish all finished");
            }
            break;

        case State::DO_PROMOTION:
            if (!current_mission_)
            {
                current_mission_ = std::make_unique<BannerMission>(nav_, action_executor_);
                current_mission_->Start(shared_from_this());
            }
            current_mission_->Update();
            if (current_mission_->GetStatus() == MissionStatus::DONE)
            {
                current_mission_.reset();
                is_banner_done_ = true;
                Transition(State::SELECT_MISSION, "Promotion finished");
            }
            else if (current_mission_->GetStatus() == MissionStatus::FAILED)
            {
                current_mission_.reset();
                is_banner_done_ = true;
                Transition(State::SELECT_MISSION, "Promotion failed");
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
