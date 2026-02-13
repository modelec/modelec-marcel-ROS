#pragma once

#include <queue>
#include <modelec_strat/action/base_action.hpp>

namespace Modelec
{
    class DownAction : public BaseAction
    {
    public:
        DownAction(const std::shared_ptr<ActionExecutor>& action_executor);
        DownAction(const std::shared_ptr<ActionExecutor>& action_executor, Side side, bool inverted = false);

        void Next() override;
        void Init(const std::vector<std::string>& params) override;
        void SetSide(Side side);
        void SetInverted(bool inverted);
        static void InitConfig();

        void End() override;

        inline static const std::string Name = ActionExec::DOWN;

    private:
        Side side_;

        bool inverted_;

        std::queue<int> steps_;

        static inline std::vector<ActionServoTimed> front_direct_msg_;
        static inline std::vector<ActionServoTimed> front_inverted_msg_;
        static inline std::vector<ActionServoTimed> back_direct_msg_;
        static inline std::vector<ActionServoTimed> back_inverted_msg_;

        static inline bool isConfigInit_ = false;

        inline static const bool registered_ =
        []()
        {
            BaseAction::Registry()[Name] =
                [](const std::shared_ptr<ActionExecutor>& exec)
                {
                    return std::make_shared<DownAction>(exec);
                };
            return true;
        }();
    };
}