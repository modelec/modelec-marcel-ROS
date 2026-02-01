#include <modelec_strat/action/down_action.hpp>

#include "modelec_strat/action_executor.hpp"

Modelec::DownAction::DownAction(const std::shared_ptr<ActionExecutor>& action_executor) : BaseAction(action_executor), side_(BOTH), inverted_(false)
{
    steps_.push(ActionExec::DOWN_STEP);
    steps_.push(ActionExec::DONE_STEP);
}

Modelec::DownAction::DownAction(const std::shared_ptr<ActionExecutor>& action_executor, Side side, bool inverted) : DownAction(action_executor)
{
    side_ = side;
    inverted_ = inverted;
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
            msg.items.resize(side_ == BOTH ? 8 : 4);

            if (side_ == FRONT || side_ == BOTH)
            {
                msg.items[0].id = 0;
                msg.items[0].start_angle = 1.76;
                msg.items[0].end_angle = 2.91;
                msg.items[0].duration_s = 0.5;

                msg.items[1].id = 1;
                msg.items[1].start_angle = 2.06;
                msg.items[1].end_angle = 0.95;
                msg.items[1].duration_s = 0.5;

                msg.items[2].id = 2;
                msg.items[2].start_angle = 0.5;
                msg.items[2].end_angle = inverted_ ? 0 : 3.2;
                msg.items[2].duration_s = 0.5;

                msg.items[3].id = 3;
                msg.items[3].start_angle = 2.6;
                msg.items[3].end_angle = inverted_ ? 3.1 : 0;
                msg.items[3].duration_s = 0.5;
            }

            if (side_ == BACK || side_ == BOTH)
            {
                int i = side_ == BOTH ? 4 : 0;
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

void Modelec::DownAction::Init(const std::vector<std::string>& params)
{
    if (!params.empty())
    {
        SetSide(static_cast<Side>(std::stoi(params[1])));
        SetInverted(params.size() >= 3 ? (params[2] == "1" || params[2] == "true") : false);
    }
}

void Modelec::DownAction::SetSide(Side side)
{
    side_ = side;
}

void Modelec::DownAction::SetInverted(bool inverted)
{
    inverted_ = inverted;
}

void Modelec::DownAction::End()
{
    if (side_ == BOTH)
    {
        action_executor_->arm_pos_[FRONT].down = true;
        action_executor_->arm_pos_[BACK].down = true;

        action_executor_->arm_pos_[FRONT].rotated = inverted_;
        action_executor_->arm_pos_[BACK].rotated = inverted_;
    }
    else
    {
        action_executor_->arm_pos_[side_].down = true;

        action_executor_->arm_pos_[side_].rotated = inverted_;
    }
}
