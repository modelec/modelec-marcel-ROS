#include <modelec_strat/action/free_action.hpp>

#include "modelec_strat/action_executor.hpp"

Modelec::FreeAction::FreeAction(const std::shared_ptr<ActionExecutor>& action_executor) : BaseAction(action_executor)
{
    steps_.push(ActionExec::FREE_STEP);
    steps_.push(ActionExec::DONE_STEP);
}

void Modelec::FreeAction::Execute()
{
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
            ActionServoPosArray msg;

            msg.items.resize(1);

            msg.items[0].id = n_ + (front_ ? 3 : 11);
            msg.items[0].angle = front_ ? 0 : 0;

            action_executor_->MoveServo(msg);
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
        SetFront(params[0] == "1" || params[0] == "true" || params[0] == "front");
        SetN(std::stoi(params[1]));
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
