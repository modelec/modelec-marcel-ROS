#include <modelec_strat/action/toggle_servo_action.hpp>

#include "modelec_strat/action_executor.hpp"

Modelec::ToggleServoAction::ToggleServoAction(const std::shared_ptr<ActionExecutor>& action_executor) : BaseAction(action_executor)
{
    steps_.push(ActionExec::TOGGLE_SERVO_STEP);
    steps_.push(ActionExec::DONE_STEP);
}

Modelec::ToggleServoAction::ToggleServoAction(const std::shared_ptr<ActionExecutor>& action_executor, Side side, int n) : ToggleServoAction(action_executor)
{
    AddServo(n, side);
}

Modelec::ToggleServoAction::ToggleServoAction(const std::shared_ptr<ActionExecutor>& action_executor, std::pair<int, Side> servo) : ToggleServoAction(action_executor)
{
    AddServo(servo.first, servo.second);
}

Modelec::ToggleServoAction::ToggleServoAction(const std::shared_ptr<ActionExecutor>& action_executor, std::vector<std::pair<int, Side>> servos) : ToggleServoAction(action_executor)
{
    AddServos(servos);
}

void Modelec::ToggleServoAction::Next()
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
    case ActionExec::TOGGLE_SERVO_STEP:
        {
            modelec_interfaces::msg::ActionServoTimedArray msg;

            msg.items.resize(servos_.size());

            for (size_t i = 0; i < servos_.size(); i++)
            {
                msg.items[i].id = servos_[i].first + (servos_[i].second ? 4 : 12);
                msg.items[i].start_angle = action_executor_->servo_pos_[servos_[i].first + (servos_[i].second ? 0 : 4)] ? 3 : 1;
                msg.items[i].end_angle = action_executor_->servo_pos_[servos_[i].first + (servos_[i].second ? 0 : 4)] ? 1 : 3;
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

void Modelec::ToggleServoAction::Init(const std::vector<std::string>& params)
{
    if (params.size() >= 2)
    {
        for (size_t i = 1; i < params.size(); i += 2)
        {
            int id = std::stoi(params[i]);
            bool side = (i + 1 < params.size()) ? (params[i + 1] == "1" || params[i + 1] == "true" || params[i + 1] == "side") : true;
            AddServo(id, side ? FRONT : BACK);
        }
    }
}

void Modelec::ToggleServoAction::AddServo(int id, Side side)
{
    servos_.emplace_back(id, side);
}

void Modelec::ToggleServoAction::AddServo(std::pair<int, Side> servo)
{
    servos_.emplace_back(servo);
}

void Modelec::ToggleServoAction::AddServos(const std::vector<std::pair<int, Side>>& servos)
{
    servos_.insert(servos_.end(), servos.begin(), servos.end());
}

void Modelec::ToggleServoAction::End()
{
    for (auto servo : servos_)
    {
        auto index = servo.first + (servo.second == FRONT ? 0 : 4);
        action_executor_->servo_pos_[index] = !action_executor_->servo_pos_[index];
    }
}
