#pragma once

#include <queue>
#include <modelec_strat/action/base_action.hpp>

namespace Modelec
{
    class DownAction : public BaseAction
    {
    public:
        DownAction(const std::shared_ptr<ActionExecutor>& action_executor);
        DownAction(const std::shared_ptr<ActionExecutor>& action_executor, Front front, bool inverted = false);

        void Next() override;
        void Init(const std::vector<std::string>& params) override;
        void SetFront(Front front);
        void SetInverted(bool inverted);

        inline static const std::string Name = ActionExec::DOWN;

    private:
        Front front_;

        bool inverted_;

        std::queue<int> steps_;

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