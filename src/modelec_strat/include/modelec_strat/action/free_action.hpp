#pragma once

#include <queue>
#include <modelec_strat/action/base_action.hpp>

namespace Modelec
{
    class FreeAction : public BaseAction
    {
    public:
        FreeAction(const std::shared_ptr<ActionExecutor>& action_executor);
        FreeAction(const std::shared_ptr<ActionExecutor>& action_executor, bool front, int n);
        FreeAction(const std::shared_ptr<ActionExecutor>& action_executor, std::pair<int, bool> servo);
        FreeAction(const std::shared_ptr<ActionExecutor>& action_executor, std::vector<std::pair<int, bool>> servos);

        void Next() override;
        void Init(const std::vector<std::string>& params) override;
        void AddServo(int id, bool front);
        void AddServo(std::pair<int, bool> servo);
        void AddServos(const std::vector<std::pair<int, bool>>& servos);

        inline static const std::string Name = ActionExec::FREE;

    private:
        std::vector<std::pair<int, bool>> servos_;

        std::queue<int> steps_;

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