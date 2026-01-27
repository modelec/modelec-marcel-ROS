#include <modelec_strat/action/take_action.hpp>

#include "modelec_strat/action_executor.hpp"

Modelec::TakeAction::TakeAction(const std::shared_ptr<ActionExecutor>& action_executor) : BaseAction(action_executor)
{
    steps_.push(ActionExec::TAKE_STEP);
    steps_.push(ActionExec::DONE_STEP);
}

Modelec::TakeAction::TakeAction(const std::shared_ptr<ActionExecutor>& action_executor, Front front, int n) : TakeAction(action_executor)
{
    AddServo(n, front);
}

Modelec::TakeAction::TakeAction(const std::shared_ptr<ActionExecutor>& action_executor, std::pair<int, Front> servo) : TakeAction(action_executor)
{
    AddServo(servo);
}

Modelec::TakeAction::TakeAction(const std::shared_ptr<ActionExecutor>& action_executor, std::vector<std::pair<int, Front>> servos) : TakeAction(action_executor)
{
    AddServos(servos);
}

void Modelec::TakeAction::Next()
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
    case ActionExec::TAKE_STEP:
        {
            modelec_interfaces::msg::ActionServoTimedArray msg;

            msg.items.resize(servos_.size());

            for (size_t i = 0; i < servos_.size(); i++)
            {
                msg.items[i].id = servos_[i].first + (servos_[i].second ? 4 : 12);
                msg.items[i].start_angle = servos_[i].second ? 1 : 0;
                msg.items[i].end_angle = servos_[i].second ? 3 : 0;
                msg.items[i].duration_s = 0.5;
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

void Modelec::TakeAction::Init(const std::vector<std::string>& params)
{
    if (params.size() >= 2)
    {
        for (size_t i = 1; i < params.size(); i += 2)
        {
            int id = std::stoi(params[i]);
            bool front = (i + 1 < params.size()) ? (params[i + 1] == "1" || params[i + 1] == "true" || params[i + 1] == "front") : true;
            AddServo(id, front ? FRONT : BACK);
        }
    }
}

void Modelec::TakeAction::AddServo(int id, Front front)
{
    servos_.emplace_back(id, front);
}

void Modelec::TakeAction::AddServo(std::pair<int, Front> servo)
{
    servos_.emplace_back(servo);
}

void Modelec::TakeAction::AddServos(const std::vector<std::pair<int, Front>>& servos)
{
    servos_.insert(servos_.end(), servos.begin(), servos.end());
}
