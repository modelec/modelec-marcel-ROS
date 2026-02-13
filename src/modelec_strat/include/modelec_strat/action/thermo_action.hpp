#pragma once

#include <queue>
#include <modelec_strat/action/base_action.hpp>

namespace Modelec
{
    class ThermoAction : public BaseAction
    {
    public:
        ThermoAction(const std::shared_ptr<ActionExecutor>& action_executor);
        ThermoAction(const std::shared_ptr<ActionExecutor>& action_executor, Side side, bool deploy);

        void Next() override;
        void Init(const std::vector<std::string>& params) override;
        void SetSide(Side side);
        void SetDeploy(bool deploy);
        static void InitConfig();

        void End() override;

        inline static const std::string Name = ActionExec::THERMO;

    private:
        Side side_ = BOTH;
        bool deploy_ = true;

        std::queue<int> steps_;

        static inline std::vector<ActionServoTimed> left_deploy_msg_;
        static inline std::vector<ActionServoTimed> left_undeploy_msg_;
        static inline std::vector<ActionServoTimed> right_deploy_msg_;
        static inline std::vector<ActionServoTimed> right_undeploy_msg_;

        static inline bool isConfigInit_ = false;

        inline static const bool registered_ =
        []()
        {
            BaseAction::Registry()[Name] =
                [](const std::shared_ptr<ActionExecutor>& exec)
                {
                    return std::make_shared<ThermoAction>(exec);
                };
            return true;
        }();
    };
}