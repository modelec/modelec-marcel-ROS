#include <modelec_strat/action/look_on_action.hpp>

#include "modelec_strat/action_executor.hpp"

Modelec::LookOnAction::LookOnAction(const std::shared_ptr<ActionExecutor>& action_executor) : BaseAction(action_executor)
{
    steps_.push(ActionExec::LOOK_ON_STEP);
    steps_.push(ActionExec::DONE_STEP);
}

Modelec::LookOnAction::LookOnAction(const std::shared_ptr<ActionExecutor>& action_executor, Side side) : LookOnAction(action_executor)
{
    side_ = side;
}

void Modelec::LookOnAction::Next()
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
    case ActionExec::LOOK_ON_STEP:
        {
            modelec_interfaces::msg::ActionServoTimedArray msg;

            msg.items.resize(1);

            msg.items[0].id = 18;
            msg.items[0].start_angle = action_executor_->cam_side_ == CENTER ? 1 : action_executor_->cam_side_ == FRONT ? 0 : 2;
            msg.items[0].end_angle = side_ == CENTER ? 1 : side_ == FRONT ? 0 : 2;
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

void Modelec::LookOnAction::Init(const std::vector<std::string>& params)
{
    if (!params.empty())
    {
        SetSide(static_cast<Side>(std::stoi(params[0])));
    }
}

void Modelec::LookOnAction::SetSide(Side side)
{
    side_ = side;
}

void Modelec::LookOnAction::End()
{
    action_executor_->cam_side_ = side_;
}
