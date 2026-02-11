#pragma once

#include <modelec_strat/missions/mission_base.hpp>
#include <modelec_strat/navigation_helper.hpp>

namespace Modelec {
    class MoveMission : virtual public Mission {
    public:
        MoveMission(const std::shared_ptr<NavigationHelper>& nav);

        static void InitConfig();

        void Start(rclcpp::Node::SharedPtr node) override;

        bool Update() override;

        std::string GetName() const override { return "Move"; }

    protected:
        std::shared_ptr<NavigationHelper> nav_;

        rclcpp::Time go_timeout_;
        rclcpp::Time last_ask_waypoint_time_;

        static double MIN_ASK_WAYPOINT;
        static double ASK_WAYPOINT_INTERVAL;
        static double GO_TIMEOUT;
        static bool IsInit;
    };
}