#pragma once

#include <modelec_strat/action/base_action.hpp>

namespace Modelec
{
    class UPAction : public BaseAction
    {
    public:
        UPAction(const ActionExecutor& action_executor);

        void Execute() override;
        std::string GetName() const override;
        void Init(const std::vector<std::string>& params) override;
        void SetFront(bool front);

    private:
        bool front_;
    };
}