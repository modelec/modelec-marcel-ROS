#pragma once

#include <rclcpp/rclcpp.hpp>

#include "action_executor.hpp"
#include "mission_manager.hpp"
#include "navigation_helper.hpp"

namespace Modelec
{

    /*
     * TODO
     * - yaml strat
     * - setup des missions
     * - gestion des interruptions (robot adverses / obstacles)
     * - scoring des missions
     *
     *
     *
     * - Banderole
     *
     *
     *
     *
     * - Gradin
     */

    class StratFMS : public rclcpp::Node
    {
    public:
        StratFMS();

        void Init();

    private:
        // rclcpp::TimerBase::SharedPtr timer_;

        std::unique_ptr<NavigationHelper> nav_;
        std::unique_ptr<MissionManager> mission_manager_;
        std::unique_ptr<ActionExecutor> action_executor_;

    };
}
