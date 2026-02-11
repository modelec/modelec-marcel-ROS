#pragma once

#include <modelec_strat/missions/mission_base.hpp>
#include <modelec_strat/action_executor.hpp>

namespace Modelec {
    class ActionMission : virtual public Mission {
    public:
        ActionMission(const std::shared_ptr<ActionExecutor>& action_executor);

        static void InitConfig();

        void Start(rclcpp::Node::SharedPtr node) override;

        bool Update() override;

        std::string GetName() const override { return "Action"; }

    protected:
        std::shared_ptr<ActionExecutor> action_executor_;
        rclcpp::Time deploy_time_;

        static double TIMEOUT;
        static bool IsInit;
    };
}