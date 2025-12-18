#include <modelec_strat/action/up_action.hpp>

#include "modelec_strat/action_executor.hpp"

Modelec::UPAction::UPAction(const std::shared_ptr<ActionExecutor>& action_executor) : BaseAction(action_executor)
{
    steps_.push(ActionExec::UP_STEP);
    steps_.push(ActionExec::DONE_STEP);
}

Modelec::UPAction::UPAction(const std::shared_ptr<ActionExecutor>& action_executor, bool front) : UPAction(action_executor)
{
    front_ = front;
}

void Modelec::UPAction::Execute()
{
}

void Modelec::UPAction::Next()
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
    case ActionExec::UP_STEP:
        {
            ActionServoTimedArray msg;
            msg.items.resize(4);

            msg.items[0].id = front_ ? 0 : 8;
            msg.items[0].start_angle = front_ ? 2.95 : 0;
            msg.items[0].end_angle = front_ ? 1.95 : 0;
            msg.items[0].duration_s = 1;

            msg.items[1].id = front_ ? 1 : 9;
            msg.items[1].start_angle = front_ ? 0.93 : 0;
            msg.items[1].end_angle = front_ ? 1.9 : 0;
            msg.items[1].duration_s = 1;

            msg.items[2].id = front_ ? 2 : 10;
            msg.items[2].start_angle = front_ ? 3.2 : 0;
            msg.items[2].end_angle = front_ ? 2.3 : 0;
            msg.items[2].duration_s = 1;

            msg.items[3].id = front_ ? 3 : 11;
            msg.items[3].start_angle = front_ ? 0 : 0;
            msg.items[3].end_angle = front_ ? 0.8 : 0;
            msg.items[3].duration_s = 1;

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

void Modelec::UPAction::Init(const std::vector<std::string>& params)
{
    if (!params.empty())
    {
        SetFront(params[1] == "1" || params[1] == "true" || params[1] == "front");
    }
}

void Modelec::UPAction::SetFront(bool front)
{
    front_ = front;
}
