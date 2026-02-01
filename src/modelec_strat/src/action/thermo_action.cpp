#include <modelec_strat/action/thermo_action.hpp>

#include "modelec_strat/action_executor.hpp"

Modelec::ThermoAction::ThermoAction(const std::shared_ptr<ActionExecutor>& action_executor) : BaseAction(action_executor)
{
    steps_.push(ActionExec::THERMO_STEP);
    steps_.push(ActionExec::DONE_STEP);
}

Modelec::ThermoAction::ThermoAction(const std::shared_ptr<ActionExecutor>& action_executor, Side side, bool deploy) : ThermoAction(action_executor)
{
    side_ = side;
    deploy_ = deploy;
}

void Modelec::ThermoAction::Next()
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
    case ActionExec::THERMO_STEP:
        {
            modelec_interfaces::msg::ActionServoTimedArray msg;

            msg.items.resize(side_ == BOTH ? 2 : 1);

            if (side_ == LEFT || side_ == BOTH)
            {
                msg.items[0].id = 16;
                msg.items[0].start_angle = deploy_ ? 1 : 2;
                msg.items[0].end_angle = deploy_ ? 2 : 1;
                msg.items[0].duration_s = 0.5;
            }

            if (side_ == RIGHT || side_ == BOTH)
            {
                msg.items[side_ == BOTH ? 1 : 0].id = 17;
                msg.items[side_ == BOTH ? 1 : 0].start_angle = deploy_ ? 1 : 2;
                msg.items[side_ == BOTH ? 1 : 0].end_angle = deploy_ ? 2 : 1;
                msg.items[side_ == BOTH ? 1 : 0].duration_s = 0.5;
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

void Modelec::ThermoAction::Init(const std::vector<std::string>& params)
{
    if (!params.empty())
    {
        SetSide(static_cast<Side>(std::stoi(params[0])));
        SetDeploy(params[1] == "1" || params[1] == "true");
    }
}

void Modelec::ThermoAction::SetSide(Side side)
{
    side_ = side;
}

void Modelec::ThermoAction::SetDeploy(bool deploy)
{
    deploy_ = deploy;
}

void Modelec::ThermoAction::End()
{
    action_executor_->thermo_state_[side_] = deploy_;
}
