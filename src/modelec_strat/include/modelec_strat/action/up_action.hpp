#pragma once

#include <queue>
#include <modelec_strat/action/base_action.hpp>

namespace Modelec
{
    class UPAction : public BaseAction
    {
    public:
        UPAction(const std::shared_ptr<ActionExecutor>& action_executor);
        UPAction(const std::shared_ptr<ActionExecutor>& action_executor, Side side);

        void Next() override;
        void Init(const std::vector<std::string>& params) override;
        void SetSide(Side side);
        static void InitConfig();

        void End() override;

        inline static const std::string Name = ActionExec::UP;

    private:
        Side side_;

        std::queue<int> steps_;

        static inline std::vector<ActionServoTimed> front_rotated_msg_;
        static inline std::vector<ActionServoTimed> front_unrotated_msg_;
        static inline std::vector<ActionServoTimed> back_rotated_msg_;
        static inline std::vector<ActionServoTimed> back_unrotated_msg_;

        static inline bool isConfigInit_ = false;

        inline static const bool registered_ =
        []()
        {
            BaseAction::Registry()[Name] =
                [](const std::shared_ptr<ActionExecutor>& exec)
                {
                    return std::make_shared<UPAction>(exec);
                };
            return true;
        }();
    };
}