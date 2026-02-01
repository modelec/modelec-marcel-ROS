#pragma once

#include <queue>
#include <modelec_strat/action/base_action.hpp>

namespace Modelec
{
    class TakeAction : public BaseAction
    {
    public:
        TakeAction(const std::shared_ptr<ActionExecutor>& action_executor);
        TakeAction(const std::shared_ptr<ActionExecutor>& action_executor, Side side, int n);
        TakeAction(const std::shared_ptr<ActionExecutor>& action_executor, std::pair<int, Side> servo);
        TakeAction(const std::shared_ptr<ActionExecutor>& action_executor, std::vector<std::pair<int, Side>> servos);

        void Next() override;
        void Init(const std::vector<std::string>& params) override;
        void AddServo(int id, Side side);
        void AddServo(std::pair<int, Side> servo);
        void AddServos(const std::vector<std::pair<int, Side>>& servos);

        void End() override;

        inline static const std::string Name = ActionExec::TAKE;

    private:
        std::vector<std::pair<int, Side>> servos_;

        std::queue<int> steps_;

        inline static const bool registered_ =
        []()
        {
            BaseAction::Registry()[Name] =
                [](const std::shared_ptr<ActionExecutor>& exec)
                {
                    return std::make_shared<TakeAction>(exec);
                };
            return true;
        }();
    };
}