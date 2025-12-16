#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <functional>
#include <vector>

#include <modelec_interfaces/msg/action_exec_new.hpp>
#include <modelec_interfaces/msg/action_servo_timed_array.hpp>
#include <modelec_interfaces/msg/action_servo_timed.hpp>

namespace Modelec
{
    class ActionExecutor;

    using ActionExec = modelec_interfaces::msg::ActionExecNew;
    using ActionServoTimedArray = modelec_interfaces::msg::ActionServoTimedArray;
    using ActionServoTimed = modelec_interfaces::msg::ActionServoTimed;

    class BaseAction
    {
    public:
        using Ptr = std::shared_ptr<BaseAction>;
        using FactoryFn =
            std::function<Ptr(const std::shared_ptr<ActionExecutor>&)>;

        static std::unordered_map<std::string, FactoryFn>& Registry()
        {
            static std::unordered_map<std::string, FactoryFn> registry;
            return registry;
        };


        BaseAction(const std::shared_ptr<ActionExecutor>& action_executor)
            : action_executor_(action_executor)
        {
        }
        virtual ~BaseAction() = default;
        virtual void Execute() = 0;
        virtual void Next() = 0;
        virtual bool IsDone() const { return done_; }
        virtual void Init(const std::vector<std::string>& params) = 0;

        // static constexpr std::string_view Name = "BaseAction";

        static Ptr CreateAction(
            const std::string& action_name,
            const std::shared_ptr<ActionExecutor>& action_executor);

    protected:
        std::shared_ptr<ActionExecutor> action_executor_;

        bool done_ = false;
    };

    inline BaseAction::Ptr BaseAction::CreateAction(const std::string& action_name,
        const std::shared_ptr<ActionExecutor>& action_executor)
    {
        auto& registry = Registry();
        auto it = registry.find(action_name);

        if (it != registry.end())
        {
            return it->second(action_executor);
        }

        return nullptr;
    }
}
