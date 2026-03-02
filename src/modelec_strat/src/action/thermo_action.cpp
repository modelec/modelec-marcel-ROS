#include <modelec_strat/action/thermo_action.hpp>

#include "modelec_strat/action_executor.hpp"

Modelec::ThermoAction::ThermoAction(const std::shared_ptr<ActionExecutor>& action_executor) : BaseAction(action_executor)
{
    steps_.push(ActionExec::THERMO_STEP);
    steps_.push(ActionExec::DONE_STEP);

    InitConfig();
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

            if (side_ == LEFT || side_ == BOTH)
            {
                if (deploy_)
                {
                    msg.items.insert(msg.items.end(), left_deploy_msg_.begin(), left_deploy_msg_.end());
                } else
                {
                    msg.items.insert(msg.items.end(), left_undeploy_msg_.begin(), left_undeploy_msg_.end());
                }
            }

            if (side_ == RIGHT || side_ == BOTH)
            {
                if (deploy_)
                {
                    msg.items.insert(msg.items.end(), right_deploy_msg_.begin(), right_deploy_msg_.end());
                } else
                {
                    msg.items.insert(msg.items.end(), right_undeploy_msg_.begin(), right_undeploy_msg_.end());
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

void Modelec::ThermoAction::InitConfig()
{
    if (isConfigInit_) return;
    isConfigInit_ = true;

    left_deploy_msg_ = Config::get<std::vector<ActionServoTimed>>("action.thermo.left.deploy");
    left_undeploy_msg_ = Config::get<std::vector<ActionServoTimed>>("action.thermo.left.undeploy");
    right_deploy_msg_ = Config::get<std::vector<ActionServoTimed>>("action.thermo.right.deploy");
    right_undeploy_msg_ = Config::get<std::vector<ActionServoTimed>>("action.thermo.right.undeploy");
}

void Modelec::ThermoAction::End()
{
    action_executor_->thermo_state_[side_] = deploy_;
}
