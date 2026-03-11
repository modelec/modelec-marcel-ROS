#pragma once

#include <queue>
#include <modelec_strat/action/base_action.hpp>

namespace Modelec
{
    class FreeAction : public BaseAction
    {
    public:
        FreeAction(const std::shared_ptr<ActionExecutor>& action_executor);
        FreeAction(const std::shared_ptr<ActionExecutor>& action_executor, Side side, int n);
        FreeAction(const std::shared_ptr<ActionExecutor>& action_executor, std::pair<int, Side> servo);
        FreeAction(const std::shared_ptr<ActionExecutor>& action_executor, std::vector<std::pair<int, Side>> servos);

        void Next() override;
        void Init(const std::vector<std::string>& params) override;
        void AddServo(int id, Side side);
        void AddServo(std::pair<int, Side> servo);
        void AddServos(const std::vector<std::pair<int, Side>>& servos);
        static void InitConfig();

        void End() override;

        inline static const std::string Name = ActionExec::FREE;

    private:
        std::vector<std::pair<int, Side>> servos_;

        std::queue<int> steps_;

        static inline int first_servo_;
        static inline int second_servo_;
        static inline double start_angle_;
        static inline double end_angle_;
        static inline double duration_s_;

        static inline bool isConfigInit_ = false;

        inline static const bool registered_ =
        []()
        {
            BaseAction::Registry()[Name] =
                [](const std::shared_ptr<ActionExecutor>& exec)
                {
                    return std::make_shared<FreeAction>(exec);
                };
            return true;
        }();
    };
}