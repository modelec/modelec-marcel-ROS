#pragma once

#include <rclcpp/rclcpp.hpp>

#include "action_executor.hpp"
#include "mission_manager.hpp"
#include "navigation_helper.hpp"
#include "missions/mission_base.hpp"
#include "missions/promotion_mission.hpp"
#include "missions/prepare_concert_mission.hpp"
#include "missions/go_home_mission.hpp"

#include "std_msgs/msg/bool.hpp"
#include <std_msgs/msg/int64.hpp>

#include "modelec_interfaces/msg/strat_state.hpp"

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

    /*
     * TODO
     * - yaml strat
     * - setup des missions
     * - gestion des interruptions (robot adverses / obstacles)
     * - scoring des missions
     */

    class StratFMS : public rclcpp::Node
    {
    public:
        StratFMS();

        void Init();

    protected:
        void transition(State next, const std::string& reason);

        void update();

    private:
        State state_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Time match_start_time_;
        bool started_ = false;
        std::unique_ptr<Mission> current_mission_;

        std::shared_ptr<NavigationHelper> nav_;
        std::unique_ptr<MissionManager> mission_manager_;
        std::unique_ptr<ActionExecutor> action_executor_;

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tirette_sub_;
        rclcpp::Publisher<modelec_interfaces::msg::StratState>::SharedPtr state_pub_;
        rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr start_time_pub_;
    };
}
