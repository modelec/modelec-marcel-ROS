#pragma once

#include <queue>
#include <modelec_strat/action/base_action.hpp>

namespace Modelec
{
    class ToggleServoAction : public BaseAction
    {
    public:
        ToggleServoAction(const std::shared_ptr<ActionExecutor>& action_executor);
        ToggleServoAction(const std::shared_ptr<ActionExecutor>& action_executor, Front front, int n);
        ToggleServoAction(const std::shared_ptr<ActionExecutor>& action_executor, std::pair<int, Front> servo);
        ToggleServoAction(const std::shared_ptr<ActionExecutor>& action_executor, std::vector<std::pair<int, Front>> servos);

        void Next() override;
        void Init(const std::vector<std::string>& params) override;
        void AddServo(int id, Front front);
        void AddServo(std::pair<int, Front> servo);
        void AddServos(const std::vector<std::pair<int, Front>>& servos);

        void End() override;

        inline static const std::string Name = ActionExec::TOGGLE_SERVO;

    private:
        std::vector<std::pair<int, Front>> servos_;

        std::queue<int> steps_;

        inline static const bool registered_ =
        []()
        {
            BaseAction::Registry()[Name] =
                [](const std::shared_ptr<ActionExecutor>& exec)
                {
                    return std::make_shared<ToggleServoAction>(exec);
                };
            return true;
        }();
    };
}