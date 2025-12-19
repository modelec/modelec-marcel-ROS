#include <modelec_strat/action/free_action.hpp>

#include "modelec_strat/action_executor.hpp"

Modelec::FreeAction::FreeAction(const std::shared_ptr<ActionExecutor>& action_executor) : BaseAction(action_executor)
{
    steps_.push(ActionExec::FREE_STEP);
    steps_.push(ActionExec::DONE_STEP);
}

Modelec::FreeAction::FreeAction(const std::shared_ptr<ActionExecutor>& action_executor, bool front, int n) : FreeAction(action_executor)
{
    front_ = front;
    n_ = n;
}

void Modelec::FreeAction::Next()
{
    if (steps_.empty())
    {
        done_ = true;
        return;
    }

    auto step = steps_.front();
    steps_.pop();

    switch (step)
    {
    case ActionExec::FREE_STEP:
        {
            modelec_interfaces::msg::ActionServoTimedArray msg;

            msg.items.resize(1);

            msg.items[0].id = n_ + (front_ ? 3 : 11);
            msg.items[0].start_angle = front_ ? 2.5 : 0;
            msg.items[0].end_angle = front_ ? 0.8 : 0;
            msg.items[0].duration_s = 0.5;
            action_executor_->MoveServoTimed(msg);
        }
        break;
    case ActionExec::DONE_STEP:
        {
            done_ = true;
        }
        break;
    default:
        break;
    }
}

void Modelec::FreeAction::Init(const std::vector<std::string>& params)
{
    if (params.size() >= 2)
    {
        SetFront(params[1] == "1" || params[1] == "true" || params[1] == "front");
        SetN(std::stoi(params[2]));
    }
}

void Modelec::FreeAction::SetFront(bool front)
{
    front_ = front;
}

void Modelec::FreeAction::SetN(int n)
{
    n_ = n;
}
