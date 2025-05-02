#include <ament_index_cpp/get_package_share_directory.hpp>
#include <modelec_strat/config.hpp>
#include <modelec_strat/strat_fms.hpp>

namespace Modelec
{

    StratFMS::StratFMS() : Node("start_fms"), state_(State::INIT)
    {
        tirette_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/tirette", 10, [this](const std_msgs::msg::Bool::SharedPtr msg)
            {
                if (!started_ && msg->data)
                {
                    started_ = true;
                }
            });

        state_pub_ = create_publisher<modelec_interfaces::msg::StratState>("/strat/state", 10);

        std::string config_path = ament_index_cpp::get_package_share_directory("modelec_strat") + "/data/config.xml";
        if (!Config::load(config_path))
        {
            RCLCPP_ERROR(get_logger(), "Failed to load config file: %s", config_path.c_str());
        }
    }

    void StratFMS::Init()
    {
        nav_ = std::make_shared<NavigationHelper>(shared_from_this());
        mission_manager_ = std::make_unique<MissionManager>(shared_from_this());
        action_executor_ = std::make_unique<ActionExecutor>(shared_from_this());

        RCLCPP_INFO(this->get_logger(), "StratFMS fully initialized");

        timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]
        {
            update();
        });
    }

    void StratFMS::transition(State next, const std::string& reason)
    {
        RCLCPP_INFO(get_logger(), "Transition %d -> %d: %s", static_cast<int>(state_), static_cast<int>(next),
                    reason.c_str());
        state_ = next;
        modelec_interfaces::msg::StratState msg;
        msg.state = static_cast<int>(state_);
        msg.reason = reason;
        state_pub_->publish(msg);
    }

    void StratFMS::update()
    {
        auto now = this->now();

        switch (state_)
        {
        case State::INIT:
            RCLCPP_INFO_ONCE(get_logger(), "State: INIT");
            transition(State::WAIT_START, "System ready");
            break;

        case State::WAIT_START:
            RCLCPP_INFO_ONCE(get_logger(), "State: WAIT_START");
            if (started_)
            {
                match_start_time_ = now;
                transition(State::SELECT_MISSION, "Match started");
            }
            break;

        case State::SELECT_MISSION:
            {
                auto elapsed = now - match_start_time_;
                // select mission in a good way there.
                if (elapsed.seconds() < 10)
                {
                    transition(State::DO_PROMOTION, "Start by preparing the concert");
                }
                else if (elapsed.seconds() < 30)
                {
                    transition(State::DO_PROMOTION, "Proceed to promotion");
                }
                else if (elapsed.seconds() >= 100.0)
                {
                    transition(State::DO_GO_HOME, "Cleanup and finish match");
                }
                else
                {
                    transition(State::STOP, "Nothing more to do");
                }
                break;
            }

        case State::DO_PREPARE_CONCERT:
            if (!current_mission_)
            {
                current_mission_ = std::make_unique<PrepareConcertMission>(nav_);
                current_mission_->start(shared_from_this());
            }
            current_mission_->update();
            if (current_mission_->getStatus() == MissionStatus::DONE)
            {
                current_mission_.reset();
                transition(State::SELECT_MISSION, "PrepareConcert finished");
            }
            break;

        case State::DO_PROMOTION:
            if (!current_mission_)
            {
                current_mission_ = std::make_unique<PromotionMission>(nav_);
                current_mission_->start(shared_from_this());
            }
            current_mission_->update();
            if (current_mission_->getStatus() == MissionStatus::DONE)
            {
                current_mission_.reset();
                transition(State::SELECT_MISSION, "Promotion finished");
            }
            break;

        case State::DO_GO_HOME:
            if (!current_mission_)
            {
                current_mission_ = std::make_unique<GoHomeMission>(nav_);
                current_mission_->start(shared_from_this());
            }
            current_mission_->update();
            if (current_mission_->getStatus() == MissionStatus::DONE)
            {
                current_mission_.reset();
                transition(State::STOP, "Cleanup done");
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
