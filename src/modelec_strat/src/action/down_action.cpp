#include <modelec_strat/action/down_action.hpp>

#include "modelec_strat/action_executor.hpp"

Modelec::DownAction::DownAction(const std::shared_ptr<ActionExecutor>& action_executor) : BaseAction(action_executor),
    side_(BOTH), inverted_(false)
{
    steps_.push(ActionExec::DOWN_STEP);
    steps_.push(ActionExec::DONE_STEP);

    InitConfig();
}

Modelec::DownAction::DownAction(const std::shared_ptr<ActionExecutor>& action_executor, Side side,
                                bool inverted) : DownAction(action_executor)
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

            if (side_ == FRONT || side_ == BOTH)
            {
                if (inverted_)
                {
                    msg.items.insert(msg.items.end(), front_inverted_msg_.begin(), front_inverted_msg_.end());
                } else
                {
                    msg.items.insert(msg.items.end(), front_direct_msg_.begin(), front_direct_msg_.end());
                }
            }

            if (side_ == BACK || side_ == BOTH)
            {
                if (inverted_)
                {
                    msg.items.insert(msg.items.end(), back_inverted_msg_.begin(), back_inverted_msg_.end());
                } else
                {
                    msg.items.insert(msg.items.end(), back_direct_msg_.begin(), back_direct_msg_.end());
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

void Modelec::DownAction::InitConfig()
{
    if (isConfigInit_) return;
    isConfigInit_ = true;

    front_direct_msg_ = Config::get<std::vector<ActionServoTimed>>("action.down.front.direct.msg");
    front_inverted_msg_ = Config::get<std::vector<ActionServoTimed>>("action.down.front.inverted.msg");
    back_direct_msg_ = Config::get<std::vector<ActionServoTimed>>("action.down.back.direct.msg");
    back_inverted_msg_ = Config::get<std::vector<ActionServoTimed>>("action.down.back.inverted.msg");
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
