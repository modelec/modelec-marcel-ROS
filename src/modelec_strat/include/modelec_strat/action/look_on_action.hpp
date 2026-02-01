#pragma once

#include <queue>
#include <modelec_strat/action/base_action.hpp>

namespace Modelec
{
    class LookOnAction : public BaseAction
    {
    public:
        LookOnAction(const std::shared_ptr<ActionExecutor>& action_executor);
        LookOnAction(const std::shared_ptr<ActionExecutor>& action_executor, Side side);

        void Next() override;
        void Init(const std::vector<std::string>& params) override;
        void SetSide(Side side);

        void End() override;

        inline static const std::string Name = ActionExec::LOOK_ON;

    private:
        Side side_ = CENTER;

        std::queue<int> steps_;

        inline static const bool registered_ =
        []()
        {
            BaseAction::Registry()[Name] =
                [](const std::shared_ptr<ActionExecutor>& exec)
                {
                    return std::make_shared<LookOnAction>(exec);
                };
            return true;
        }();
    };
}