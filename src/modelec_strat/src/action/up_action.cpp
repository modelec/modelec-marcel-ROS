#include <modelec_strat/action/up_action.hpp>

#include "modelec_strat/action_executor.hpp"

Modelec::UPAction::UPAction(const std::shared_ptr<ActionExecutor>& action_executor) : BaseAction(action_executor)
{
    steps_.push(ActionExec::UP_STEP);
    steps_.push(ActionExec::DONE_STEP);

    InitConfig();
}

Modelec::UPAction::UPAction(const std::shared_ptr<ActionExecutor>& action_executor, Side side) : UPAction(action_executor)
{
    side_ = side;
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

            if (side_ == FRONT || side_ == BOTH)
            {
                if (action_executor_->arm_pos_[FRONT].rotated)
                {
                    msg.items.insert(msg.items.end(), front_rotated_msg_.begin(), front_rotated_msg_.end());
                } else
                {
                    msg.items.insert(msg.items.end(), front_unrotated_msg_.begin(), front_unrotated_msg_.end());
                }
            }

            if (side_ == BACK || side_ == BOTH)
            {
                if (action_executor_->arm_pos_[BACK].rotated)
                {
                    msg.items.insert(msg.items.end(), back_rotated_msg_.begin(), back_rotated_msg_.end());
                } else
                {
                    msg.items.insert(msg.items.end(), back_unrotated_msg_.begin(), back_unrotated_msg_.end());
                }
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
        SetSide(static_cast<Side>(std::stoi(params[1])));
    }
}

void Modelec::UPAction::SetSide(Side side)
{
    side_ = side;
}

void Modelec::UPAction::InitConfig()
{
    if (isConfigInit_) return;
    isConfigInit_ = true;

    front_rotated_msg_ = Config::get<std::vector<ActionServoTimed>>("action.up.front.rotated.msg");
    front_unrotated_msg_ = Config::get<std::vector<ActionServoTimed>>("action.up.front.unrotated.msg");
    back_rotated_msg_ = Config::get<std::vector<ActionServoTimed>>("action.up.back.rotated.msg");
    back_unrotated_msg_ = Config::get<std::vector<ActionServoTimed>>("action.up.back.unrotated.msg");
}

void Modelec::UPAction::End()
{
    if (side_ == BOTH)
    {
        action_executor_->arm_pos_[FRONT].down = false;
        action_executor_->arm_pos_[BACK].down = false;
    }
    else
    {
        action_executor_->arm_pos_[side_].down = false;
    }
}
