#include <modelec_strat/action/down_action.hpp>

#include "modelec_strat/action_executor.hpp"

Modelec::DownAction::DownAction(const std::shared_ptr<ActionExecutor>& action_executor) : BaseAction(action_executor)
{
    steps_.push(ActionExec::DOWN_STEP);
    steps_.push(ActionExec::DONE_STEP);
}

Modelec::DownAction::DownAction(const std::shared_ptr<ActionExecutor>& action_executor, Front front) : DownAction(action_executor)
{
    front_ = front;
}

void Modelec::DownAction::Next()
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
    case ActionExec::DOWN_STEP:
        {
            ActionServoTimedArray msg;
            msg.items.resize(front_ == BOTH ? 8 : 4);

            if (front_ == FRONT || front_ == BOTH)
            {
                msg.items[0].id = 0;
                msg.items[0].start_angle = 1.95;
                msg.items[0].end_angle = 2.95;
                msg.items[0].duration_s = 1;

                msg.items[1].id = 1;
                msg.items[1].start_angle = 1.9;
                msg.items[1].end_angle = 0.9;
                msg.items[1].duration_s = 1;

                msg.items[2].id = 2;
                msg.items[2].start_angle = 0.3;
                msg.items[2].end_angle = 0;
                msg.items[2].duration_s = 1;

                msg.items[3].id = 3;
                msg.items[3].start_angle = 2.7;
                msg.items[3].end_angle = 3;
                msg.items[3].duration_s = 1;
            }

            if (front_ == BACK || front_ == BOTH)
            {
                int i = front_ == BOTH ? 4 : 0;
                msg.items[i].id = 8;
                msg.items[i].start_angle = 0;
                msg.items[i].end_angle = 0;
                msg.items[i].duration_s = 1;

                msg.items[i+1].id = 9;
                msg.items[i+1].start_angle = 0;
                msg.items[i+1].end_angle = 0;
                msg.items[i+1].duration_s = 1;

                msg.items[i+2].id = 10;
                msg.items[i+2].start_angle = 0;
                msg.items[i+2].end_angle = 0;
                msg.items[i+2].duration_s = 1;

                msg.items[i+3].id = 11;
                msg.items[i+3].start_angle = 0;
                msg.items[i+3].end_angle = 0;
                msg.items[i+3].duration_s = 1;
            }

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

void Modelec::DownAction::Init(const std::vector<std::string>& params)
{
    if (!params.empty())
    {
        SetFront(static_cast<Front>(std::stoi(params[1])));
    }
}

void Modelec::DownAction::SetFront(Front front)
{
    front_ = front;
}
