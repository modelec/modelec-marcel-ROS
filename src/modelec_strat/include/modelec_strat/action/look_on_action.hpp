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
        static void InitConfig();

        void End() override;

        inline static const std::string Name = ActionExec::LOOK_ON;

    private:
        Side side_ = CENTER;

        std::queue<int> steps_;

        static inline int id_;
        static inline double center_;
        static inline double front_;
        static inline double back_;
        static inline double duration_s_;

        static inline bool isConfigInit_ = false;

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