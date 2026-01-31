#include <modelec_strat/action/up_action.hpp>

#include "modelec_strat/action_executor.hpp"

Modelec::UPAction::UPAction(const std::shared_ptr<ActionExecutor>& action_executor) : BaseAction(action_executor)
{
    steps_.push(ActionExec::UP_STEP);
    steps_.push(ActionExec::DONE_STEP);
}

Modelec::UPAction::UPAction(const std::shared_ptr<ActionExecutor>& action_executor, Front front) : UPAction(action_executor)
{
    front_ = front;
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
            msg.items.resize(front_ == BOTH ? 8 : 4);

            if (front_ == FRONT || front_ == BOTH)
            {
                msg.items[0].id = 0;
                msg.items[0].start_angle = 2.91;
                msg.items[0].end_angle = 1.76;
                msg.items[0].duration_s = 0.5;

                msg.items[1].id = 1;
                msg.items[1].start_angle = 0.95;
                msg.items[1].end_angle = 2.06;
                msg.items[1].duration_s = 0.5;

                msg.items[2].id = 2;
                msg.items[2].start_angle = action_executor_->arm_pos_[FRONT].rotated ? 0 : 3.2;
                msg.items[2].end_angle = 0.5;
                msg.items[2].duration_s = 0.5;

                msg.items[3].id = 3;
                msg.items[3].start_angle = action_executor_->arm_pos_[FRONT].rotated ? 3.1 : 0;
                msg.items[3].end_angle = 2.6;
                msg.items[3].duration_s = 0.5;
            }

            if (front_ == BACK || front_ == BOTH) {
                int i = front_ == BOTH ? 4 : 0;

                msg.items[i].id = 8;
                msg.items[i].start_angle = 0;
                msg.items[i].end_angle = 0;
                msg.items[i].duration_s = 0.5;

                msg.items[i+1].id = 9;
                msg.items[i+1].start_angle = 0;
                msg.items[i+1].end_angle = 0;
                msg.items[i+1].duration_s = 0.5;

                msg.items[i+2].id = 10;
                msg.items[i+2].start_angle = 0;
                msg.items[i+2].end_angle = 0;
                msg.items[i+2].duration_s = 0.5;

                msg.items[i+3].id = 11;
                msg.items[i+3].start_angle = 0;
                msg.items[i+3].end_angle = 0;
                msg.items[i+3].duration_s = 0.5;
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

void Modelec::UPAction::Init(const std::vector<std::string>& params)
{
    if (!params.empty())
    {
        SetFront(static_cast<Front>(std::stoi(params[1])));
    }
}

void Modelec::UPAction::SetFront(Front front)
{
    front_ = front;
}

void Modelec::UPAction::End()
{
    if (front_ == BOTH)
    {
        action_executor_->arm_pos_[FRONT].down = false;
        action_executor_->arm_pos_[BACK].down = false;
    }
    else
    {
        action_executor_->arm_pos_[front_].down = false;
    }
}
