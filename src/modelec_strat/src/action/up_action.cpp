#include <modelec_strat/action/up_action.hpp>

Modelec::UPAction::UPAction(const ActionExecutor& action_executor) : BaseAction(action_executor)
{
}

void Modelec::UPAction::Execute()
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

    // action_executor_->something(msg);
}

std::string Modelec::UPAction::GetName() const
{
    return ActionExecNewMsg::UP;
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
