#pragma once

#include <modelec_strat/missions/mission_base.hpp>
#include <optional>

namespace Modelec {
    class MinTimeMission : virtual public Mission {
    public:
        MinTimeMission();

        static void InitConfig();

        void Start(const rclcpp::Node::SharedPtr& node) override;

        bool Update() override;

        std::string GetName() const override { return "MinTime"; }

    protected:
        std::optional<rclcpp::Time> min_time_;

        static double MIN_TIME_DURATION;
        static bool IsInit;
    };
}