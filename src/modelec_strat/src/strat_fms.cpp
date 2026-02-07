#include <modelec_strat/strat_fms.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <modelec_utils/config.hpp>

#include <modelec_strat/missions/go_home_mission.hpp>
#include <modelec_strat/missions/take_mission.hpp>
#include <modelec_strat/missions/free_mission.hpp>
#include <modelec_strat/missions/thermo_mission.hpp>

#include "modelec_strat/action/base_action.hpp"

namespace Modelec
{

    StratFMS::StratFMS() : Node("start_fms")
    {
        this->declare_parameter<bool>("static_strat", false);
        this->declare_parameter<double>("factor.obs", 1.0);
        this->declare_parameter<double>("factor.thermo", 0.5);
        this->declare_parameter<int>("timer_period_ms", 100);

        static_strat_ = this->get_parameter("static_strat").as_bool();
        factor_obs_ = this->get_parameter("factor.obs").as_double();
        factor_thermo_ = this->get_parameter("factor.thermo").as_double();
        timer_period_ms_ = this->get_parameter("timer_period_ms").as_int();

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
                nav_->SetTeamId(static_cast<NavigationHelper::Team>(msg->data));
            });

        spawn_id_sub_ = create_subscription<modelec_interfaces::msg::Spawn>(
            "/strat/spawn", 10, [this](const modelec_interfaces::msg::Spawn::SharedPtr msg)
            {
                team_selected_ = true;
                team_id_ = msg->team_id;
                nav_->SetTeamId(static_cast<NavigationHelper::Team>(team_id_));
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

        timer_ = create_wall_timer(std::chrono::milliseconds(timer_period_ms_), [this]
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

                action_executor_->Up(BaseAction::Side::BOTH, true);
                action_executor_->Free({
                    {0, BaseAction::FRONT}, {1, BaseAction::FRONT}, {2, BaseAction::FRONT}, {3, BaseAction::FRONT},
                    {0, BaseAction::BACK}, {1, BaseAction::BACK}, {2, BaseAction::BACK}, {3, BaseAction::BACK},
                });

                auto empty_queue_ = std::queue<State>();
                std::swap(game_action_sequence_, empty_queue_);
                game_action_sequence_.push(State::TAKE_MISSION);
                game_action_sequence_.push(State::TAKE_MISSION);
                game_action_sequence_.push(State::FREE_MISSION);
                game_action_sequence_.push(State::FREE_MISSION);
                game_action_sequence_.push(State::THERMO_MISSION);

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
                auto duration = (now - match_start_time_).seconds();

                if (duration >= 100)
                {
                    Transition(State::STOP, "All missions done");
                }

                else if (duration > 60 && !action_executor_->IsEmpty() && duration < 90)
                {
                    Transition(State::FREE_MISSION, "No Time left, freeing boxes");
                }
                else if (duration < 80)
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

				if (static_strat_) {
					if (game_action_sequence_.empty()) {
						Transition(State::DO_GO_HOME, "No more game actions in sequence");
						return;
					}

					auto next_action = game_action_sequence_.front();
					game_action_sequence_.pop();

					Transition(next_action, "Selecting next game action from sequence");
					return;
				}

                auto pos = nav_->GetCurrentPos();
                auto closestBox = nav_->GetClosestObstacle<BoxObstacle>(pos);
                auto closestDeposite = nav_->GetClosestDepositeZone(pos);

                auto thermoPos = nav_->GetThermoPositions()[0];

                double distToBox = Point::distance(Point(pos->x, pos->y, pos->theta),
                                            closestBox->GetPosition()) * factor_obs_;
                double distToDeposite = Point::distance(Point(pos->x, pos->y, pos->theta),
                                            closestDeposite->GetPosition());
                double distToThermo = Point::distance(Point(pos->x, pos->y, pos->theta),
                                            thermoPos) * factor_thermo_;

                if (distToThermo < distToBox && distToThermo < distToDeposite && !ThermoMission::IsThermoDone)
                {
                    RCLCPP_INFO(get_logger(), "Choosing THERMO mission (dist to thermo: %.2f < dist to box: %.2f, dist to deposite: %.2f)",
                                distToThermo, distToBox, distToDeposite);
                    Transition(State::THERMO_MISSION, "Selecting THERMO mission");
                } else
                {
                    if (action_executor_->IsFull())
                    {
                        RCLCPP_INFO(get_logger(), "All box are present on robot, selecting FREE mission");
                        Transition(State::FREE_MISSION, "Selecting FREE mission");
                    }
                    else if (action_executor_->IsEmpty())
                    {
                        RCLCPP_INFO(get_logger(), "No box present on robot, selecting TAKE mission");
                        Transition(State::TAKE_MISSION, "Selecting TAKE mission");
                    }
                    else
                    {

                        if (closestBox && closestDeposite)
                        {
                            if (distToBox < distToDeposite)
                            {
                                RCLCPP_INFO(get_logger(), "Choosing TAKE mission (dist to box: %.2f < dist to deposite: %.2f)",
                                            distToBox, distToDeposite);
                                Transition(State::TAKE_MISSION, "Selecting TAKE mission");
                            }
                            else
                            {
                                RCLCPP_INFO(get_logger(), "Choosing FREE mission (dist to deposite: %.2f < dist to box: %.2f)",
                                            distToDeposite, distToBox);
                                Transition(State::FREE_MISSION, "Selecting FREE mission");
                            }
                        }
                        else if (closestBox)
                        {
                            RCLCPP_INFO(get_logger(), "Only box available, selecting TAKE mission");
                            Transition(State::TAKE_MISSION, "Selecting TAKE mission");
                        }
                        else if (closestDeposite)
                        {
                            RCLCPP_INFO(get_logger(), "Only deposite available, selecting FREE mission");
                            Transition(State::FREE_MISSION, "Selecting FREE mission");
                        }
                        else
                        {
                            RCLCPP_WARN(get_logger(), "No box or deposite available, cannot select mission!");
                            Transition(State::STOP, "No missions available");
                        }
                    }
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
			else if (current_mission_->GetStatus() == MissionStatus::FAILED)
            {
                current_mission_.reset();
                RCLCPP_ERROR(get_logger(), "Take mission failed!");
                Transition(State::SELECT_MISSION, "Take mission failed");
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
			else if (current_mission_->GetStatus() == MissionStatus::FAILED)
            {
                current_mission_.reset();
                RCLCPP_ERROR(get_logger(), "Free mission failed!");
                Transition(State::SELECT_MISSION, "Free mission failed");
            }
            break;

        case State::THERMO_MISSION:
            {
                if (!current_mission_)
                {
                    current_mission_ = std::make_unique<ThermoMission>(nav_, action_executor_);
                    current_mission_->Start(shared_from_this());
                }
                current_mission_->Update();
                if (current_mission_->GetStatus() == MissionStatus::DONE)
                {
                    current_mission_.reset();
                    Transition(State::SELECT_MISSION, "Thermo done");
                }
                else if (current_mission_->GetStatus() == MissionStatus::FAILED)
                {
                    current_mission_.reset();
                    RCLCPP_ERROR(get_logger(), "Thermo mission failed!");
                    Transition(State::SELECT_MISSION, "Thermo mission failed");
                }
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
			else if (current_mission_->GetStatus() == MissionStatus::FAILED)
            {
                current_mission_.reset();
                RCLCPP_ERROR(get_logger(), "Go Home mission failed!");
                Transition(State::STOP, "Go Home mission failed");
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
