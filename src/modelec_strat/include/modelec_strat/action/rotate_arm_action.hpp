#pragma once

#include <queue>
#include <modelec_strat/action/base_action.hpp>

namespace Modelec
{
    class RotateArmAction : public BaseAction
    {
    public:
        RotateArmAction(const std::shared_ptr<ActionExecutor>& action_executor);
        RotateArmAction(const std::shared_ptr<ActionExecutor>& action_executor, Front front, bool rotated = false);

        void Next() override;
        void Init(const std::vector<std::string>& params) override;
        void SetFront(Front front);
        void SetRotated(bool rotated);

        inline static const std::string Name = ActionExec::TOGGLE_ARM;

    private:
        Front front_;

        bool rotated_ = false;

        std::queue<int> steps_;

        inline static const bool registered_ =
        []()
        {
            BaseAction::Registry()[Name] =
                [](const std::shared_ptr<ActionExecutor>& exec)
                {
                    return std::make_shared<RotateArmAction>(exec);
                };
            return true;
        }();
    };
}