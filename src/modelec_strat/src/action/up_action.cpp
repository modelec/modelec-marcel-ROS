#include <modelec_strat/action/up_action.hpp>

#include "modelec_strat/action_executor.hpp"

Modelec::UPAction::UPAction(const std::shared_ptr<ActionExecutor>& action_executor) : BaseAction(action_executor)
{
    steps_.push(ActionExec::UP_STEP);
    steps_.push(ActionExec::DONE_STEP);
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

            msg.items[0].id = 0;
            msg.items[0].start_angle = 0;
            msg.items[0].end_angle = 1.49;
            msg.items[0].duration_s = 10;

            msg.items[1].id = 1;
            msg.items[1].start_angle = 3;
            msg.items[1].end_angle = 1.5;
            msg.items[1].duration_s = 10;

            msg.items[2].id = 4;
            msg.items[2].start_angle = 1.45;
            msg.items[2].end_angle = 3;
            msg.items[2].duration_s = 10;

            msg.items[3].id = 5;
            msg.items[3].start_angle = 1.6;
            msg.items[3].end_angle = 0;
            msg.items[3].duration_s = 10;

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
        SetFront(params[0] == "1" || params[0] == "true" || params[0] == "front");
    }
}

void Modelec::UPAction::SetFront(bool front)
{
    front_ = front;
}
