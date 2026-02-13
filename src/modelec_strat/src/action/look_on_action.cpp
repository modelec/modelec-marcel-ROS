#include <modelec_strat/action/look_on_action.hpp>

#include "modelec_strat/action_executor.hpp"

Modelec::LookOnAction::LookOnAction(const std::shared_ptr<ActionExecutor>& action_executor) : BaseAction(action_executor)
{
    steps_.push(ActionExec::LOOK_ON_STEP);
    steps_.push(ActionExec::DONE_STEP);

    InitConfig();
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

            msg.items[0].id = id_;
            msg.items[0].start_angle = action_executor_->cam_side_ == CENTER ? center_ : action_executor_->cam_side_ == FRONT ? front_ : back_;
            msg.items[0].end_angle = side_ == CENTER ? center_ : side_ == FRONT ? front_ : back_;
            msg.items[0].duration_s = duration_s_;

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

void Modelec::LookOnAction::InitConfig()
{
    if (isConfigInit_) return;
    isConfigInit_ = true;

    id_ = Config::get<int>("action.look_on.id", 1);
    center_ = Config::get<double>("action.look_on.center", 1);
    front_ = Config::get<double>("action.look_on.front", 0);
    back_ = Config::get<double>("action.look_on.back", 2);
    duration_s_ = Config::get<double>("action.look_on.duration_s", 0.5);
}

void Modelec::LookOnAction::End()
{
    action_executor_->cam_side_ = side_;
}
