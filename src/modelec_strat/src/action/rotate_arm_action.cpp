#include <modelec_strat/action/rotate_arm_action.hpp>

#include "modelec_strat/action_executor.hpp"

Modelec::RotateArmAction::RotateArmAction(const std::shared_ptr<ActionExecutor>& action_executor) : BaseAction(action_executor)
{
    steps_.push(ActionExec::ROTATE_ARM_STEP);
    steps_.push(ActionExec::DONE_STEP);
}

Modelec::RotateArmAction::RotateArmAction(const std::shared_ptr<ActionExecutor>& action_executor, Front front, bool rotated)
    : RotateArmAction(action_executor)
{
    front_ = front;
    rotated_ = rotated;
}

void Modelec::RotateArmAction::Next()
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
    case ActionExec::ROTATE_ARM_STEP:
        {
            ActionServoTimedArray msg;
            msg.items.resize(2);

            rotated_ = !rotated_;

            if (rotated_)
            {
                msg.items[0].id = 2 + front_ == FRONT ? 0 : 8;
                msg.items[0].start_angle = front_ == BaseAction::FRONT ? 0.2 : 0;
                msg.items[0].end_angle = front_ == BaseAction::FRONT ? 0 : 0;
                msg.items[0].duration_s = 1;

                msg.items[1].id = 3 + front_ == FRONT ? 0 : 8;
                msg.items[1].start_angle = front_ == BaseAction::FRONT ? 2.8 : 0;
                msg.items[1].end_angle = front_ == BaseAction::FRONT ? 0 : 0;
                msg.items[1].duration_s = 1;
            } else
            {
                msg.items[0].id = 2 + front_ == FRONT ? 0 : 8;
                msg.items[0].start_angle = front_ == BaseAction::FRONT ? 0 : 0;
                msg.items[0].end_angle = front_ == BaseAction::FRONT ? 0.2 : 0;
                msg.items[0].duration_s = 1;

                msg.items[1].id = 3 + front_ == FRONT ? 0 : 8;
                msg.items[1].start_angle = front_ == BaseAction::FRONT ? 0 : 0;
                msg.items[1].end_angle = front_ == BaseAction::FRONT ? 2.8 : 0;
                msg.items[1].duration_s = 1;
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

void Modelec::RotateArmAction::Init(const std::vector<std::string>& params)
{
    if (!params.empty())
    {
        SetFront(static_cast<Front>(std::stoi(params[1])));
        SetRotated(params.size() >= 3 ? (params[2] == "1" || params[2] == "true") : false);
    }
}

void Modelec::RotateArmAction::SetFront(Front front)
{
    front_ = front;
}

void Modelec::RotateArmAction::SetRotated(bool rotated)
{
    rotated_ = rotated;
}