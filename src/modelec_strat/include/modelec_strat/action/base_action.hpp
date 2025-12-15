#pragma once

#include <memory>
#include <string>

#include <modelec_interfaces/msg/action_exec_new.hpp>
#include <modelec_interfaces/msg/action_servo_timed_array.hpp>
#include <modelec_interfaces/msg/action_servo_timed.hpp>

namespace Modelec
{
    class ActionExecutor;

    using ActionExecNewMsg = modelec_interfaces::msg::ActionExecNew;
    using ActionServoTimedArray = modelec_interfaces::msg::ActionServoTimedArray;
    using ActionServoTimed = modelec_interfaces::msg::ActionServoTimed;

    class BaseAction
    {
    public:
        BaseAction(const std::shared_ptr<ActionExecutor>& action_executor)
            : action_executor_(action_executor)
        {
        }
        virtual ~BaseAction() = default;
        virtual void Execute() = 0;
        virtual bool IsDone() const = 0;
        virtual std::string GetName() const = 0;
        virtual void Init(const std::vector<std::string>& params) = 0;

    protected:
        std::shared_ptr<ActionExecutor> action_executor_;

        bool done_ = false;
    };
}