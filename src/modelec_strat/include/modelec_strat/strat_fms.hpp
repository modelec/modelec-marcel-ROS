#pragma once

#include <rclcpp/rclcpp.hpp>

#include "action_executor.hpp"
#include "navigation_helper.hpp"
#include "missions/mission_base.hpp"
#include "missions/banner_mission.hpp"
#include "missions/prepare_concert_mission.hpp"
#include "missions/go_home_mission.hpp"

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/int8.hpp>

#include <modelec_interfaces/msg/strat_state.hpp>
#include <modelec_interfaces/msg/spawn.hpp>

#include <modelec_interfaces/srv/odometry_start.hpp>
#include <std_msgs/msg/empty.hpp>


namespace Modelec
{
    enum class State
    {
        INIT,
        WAIT_START,
        SELECT_MISSION,

        DO_PREPARE_CONCERT,
        DO_PROMOTION,

        DO_GO_HOME,
        STOP
    };

    class StratFMS : public rclcpp::Node
    {
    public:
        StratFMS();

        void Init();

        void ReInit();

        void Reset();

        void ResetStrat();

    protected:
        void Transition(State next, const std::string& reason);

        void Update();

    private:
        State state_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Time match_start_time_;
        bool started_ = false;
        bool setup_ = false;
        std::unique_ptr<Mission> current_mission_;
        int team_id_ = 0;
        bool is_banner_done_ = false;

        std::shared_ptr<NavigationHelper> nav_;
        std::shared_ptr<ActionExecutor> action_executor_;

        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr tir_sub_;
        rclcpp::Publisher<modelec_interfaces::msg::StratState>::SharedPtr state_pub_;
        rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr start_time_pub_;
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr team_id_sub_;
        rclcpp::Subscription<modelec_interfaces::msg::Spawn>::SharedPtr spawn_id_sub_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_strat_sub_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr tir_arm_sub_;

        rclcpp::Client<modelec_interfaces::srv::OdometryStart>::SharedPtr client_start_;
    };
}
