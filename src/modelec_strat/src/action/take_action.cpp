#include <modelec_strat/action/take_action.hpp>

#include "modelec_strat/action_executor.hpp"

Modelec::TakeAction::TakeAction(const std::shared_ptr<ActionExecutor>& action_executor) : BaseAction(action_executor)
{
}

Modelec::TakeAction::TakeAction(const std::shared_ptr<ActionExecutor>& action_executor, bool front, int n) : TakeAction(action_executor)
{
    front_ = front;
    n_ = n;
}

void Modelec::TakeAction::Next()
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
    case ActionExec::TAKE_STEP:
        {
            modelec_interfaces::msg::ActionServoTimedArray msg;

            msg.items.resize(1);

            msg.items[0].id = n_ + (front_ ? 3 : 11);
            msg.items[0].start_angle = front_ ? 0.8 : 0;
            msg.items[0].end_angle = front_ ? 2.7 : 0;
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

void Modelec::TakeAction::Init(const std::vector<std::string>& params)
{
    if (params.size() >= 2)
    {
        SetFront(params[1] == "1" || params[1] == "true" || params[1] == "front");
        SetN(std::stoi(params[2]));
    }
}

void Modelec::TakeAction::Init(bool front, int n)
{
}

void Modelec::TakeAction::SetFront(bool front)
{
    front_ = front;
}

void Modelec::TakeAction::SetN(int n)
{
    n_ = n;
}
