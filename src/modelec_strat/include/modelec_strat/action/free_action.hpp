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

        void Execute() override;
        void Next() override;
        void Init(const std::vector<std::string>& params) override;
        void SetFront(bool front);
        void SetN(int n);

        inline static const std::string Name = ActionExec::FREE;

    private:
        bool front_;
        int n_;

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