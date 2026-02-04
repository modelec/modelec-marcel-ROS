#include <modelec_strat/action_executor.hpp>

#include "modelec_strat/action/down_action.hpp"
#include "modelec_strat/action/up_action.hpp"
#include "modelec_strat/action/free_action.hpp"
#include "modelec_strat/action/take_action.hpp"
#include "modelec_strat/action/toggle_servo_action.hpp"
#include "modelec_strat/action/look_on_action.hpp"
#include "modelec_strat/action/thermo_action.hpp"
#include "modelec_utils/utils.hpp"

namespace Modelec
{
    ActionExecutor::ActionExecutor()
    {
    }

    ActionExecutor::ActionExecutor(const rclcpp::Node::SharedPtr& node) : node_(node)
    {
        asc_move_pub_ = node_->create_publisher<modelec_interfaces::msg::ActionAscPos>("/action/move/asc", 10);
        servo_move_pub_ = node_->create_publisher<modelec_interfaces::msg::ActionServoPosArray>("/action/move/servo", 10);
        relay_move_pub_ = node_->create_publisher<modelec_interfaces::msg::ActionRelayStateArray>("/action/move/relay", 10);

        asc_move_res_sub_ = node_->create_subscription<modelec_interfaces::msg::ActionAscPos>(
            "/action/move/asc/res", 10, [this](const modelec_interfaces::msg::ActionAscPos::SharedPtr)
            {
                step_running_--;
                Update();
            });

        servo_move_res_sub_ = node_->create_subscription<modelec_interfaces::msg::ActionServoPosArray>(
            "/action/move/servo/res", 10, [this](const modelec_interfaces::msg::ActionServoPosArray::SharedPtr msg)
            {
                for (const auto& item : msg->items)
                {
                    servo_value_[item.id] = item.angle;
                }
            });

        relay_move_res_sub_ = node_->create_subscription<modelec_interfaces::msg::ActionRelayStateArray>(
            "/action/move/relay/res", 10, [this](const modelec_interfaces::msg::ActionRelayStateArray::SharedPtr msg)
            {
                step_running_ -= msg->items.size();
                Update();
            });

        action_exec_sub_ = node_->create_subscription<modelec_interfaces::msg::ActionExec>(
            "/action/exec", 10, [this](const modelec_interfaces::msg::ActionExec::SharedPtr msg)
            {
                auto token = split(msg->action, ActionExec::DELIMITER[0]);

                if (token.empty())
                {
                    RCLCPP_WARN(
                        node_->get_logger(),
                        "Empty action received");
                    return;
                }

                auto action = BaseAction::CreateAction(
                    token[0],
                    shared_from_this());
                if (action == nullptr)
                {
                    RCLCPP_WARN(
                        node_->get_logger(),
                        "Unknown action: %s",
                        msg->action.c_str());
                    return;
                }
                action->Init(token);
                action_.push(action);
                action_done_ = false;
                step_running_ = 0;
                Update();
            });

        servo_timed_move_pub_ = node_->create_publisher<modelec_interfaces::msg::ActionServoTimedArray>("/action/move/servo/timed", 10);

        servo_timed_move_res_sub_ = node_->create_subscription<modelec_interfaces::msg::ActionServoTimedArray>(
            "/action/move/servo/timed/res", 10, [this](const modelec_interfaces::msg::ActionServoTimedArray::SharedPtr msg)
            {
                step_running_ -= msg->items.size();
                Update();
            });

        joy_sub_ = node_->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, [this](const sensor_msgs::msg::Joy::SharedPtr msg)
            {
                // use game controller to manually control all the action. make it carefully
                if (msg->buttons.size() >= 15)
                {
                    if (msg->buttons[0] == 1) // A button
                    {
                        if (action_done_)
                        {
                            ToggleServo({{0, BaseAction::FRONT}});
                        }
                    }
                    else if (msg->buttons[1] == 1) // B button
                    {
                        if (action_done_)
                        {
                            ToggleServo({{1, BaseAction::FRONT}});
                        }
                    }
                    else if (msg->buttons[3] == 1) // X button
                    {
                        if (action_done_)
                        {
                            ToggleServo({{3, BaseAction::FRONT}});
                        }
                    }
                    else if (msg->buttons[4] == 1) // Y button
                    {
                        if (action_done_)
                        {
                            ToggleServo({{2, BaseAction::FRONT}});
                        }
                    }
                    else if (msg->buttons[6] == 1) // L1 button
                    {
                        if (action_done_)
                        {
                            RotateArm(BaseAction::BACK, false, !arm_pos_[BaseAction::BACK].rotated);
                        }
                    }
                    else if (msg->buttons[7] == 1) // R1 button
                    {
                        if (action_done_)
                        {
                            RotateArm(BaseAction::FRONT, false, !arm_pos_[BaseAction::FRONT].rotated);
                        }
                    }
                    else if (msg->buttons[14] == 1) // LT button
                    {
                        ActivateThermo(BaseAction::Side::LEFT, !thermo_state_[BaseAction::Side::LEFT]);
                    }
                    else if (msg->buttons[15] == 1) // LR button
                    {
                        ActivateThermo(BaseAction::Side::RIGHT, !thermo_state_[BaseAction::Side::RIGHT]);
                    }
                }
                if (msg->axes.size() == 8)
                {
                    auto left_trig = msg->axes[4];
                    auto right_trig = msg->axes[5];

                    if (left_trig == 1 && last_left_trig == -1) // left trigger pressed
                    {
                        if (action_done_)
                        {
                            ToggleArm(BaseAction::BACK);
                        }
                        last_left_trig = left_trig;
                    } else if (left_trig == -1 && last_left_trig == 1) // left trigger released
                    {
                        last_left_trig = left_trig;
                    }

                    if (right_trig == 1 && last_left_trig == -1) // right trigger pressed
                    {
                        if (action_done_)
                        {
                            ToggleArm(BaseAction::FRONT);
                        }
                        last_right_trig = right_trig;
                    } else if (right_trig == -1 && last_right_trig == 1) // right trigger released
                    {
                        last_right_trig = right_trig;
                    }

                    auto btn_horizontal = msg->axes[6];
                    auto btn_vertical = msg->axes[7];

                    if (btn_horizontal == 1.0) // left
                    {
                        if (action_done_)
                        {
                            ToggleServo({{3, BaseAction::BACK}});
                        }
                    }
                    else if (btn_horizontal == -1.0) // right
                    {
                        if (action_done_)
                        {
                            ToggleServo({{1, BaseAction::BACK}});
                        }
                    }
                    if (btn_vertical == 1.0) // up
                    {
                        if (action_done_)
                        {
                            ToggleServo({{0, BaseAction::BACK}});
                        }
                    }
                    else if (btn_vertical == -1.0) // down
                    {
                        if (action_done_)
                        {
                            ToggleServo({{2, BaseAction::BACK}});
                        }
                    }
                }
            });

        score_pub_ = node_->create_publisher<std_msgs::msg::Int64>("/strat/score", 10);
    }

    rclcpp::Node::SharedPtr ActionExecutor::GetNode() const
    {
        return node_;
    }

    bool ActionExecutor::IsActionDone() const
    {
        return action_done_;
    }

    void ActionExecutor::Update()
    {
        auto action = action_.front();
        if (action != nullptr && !action->IsDone() && step_running_ <= 0)
        {
            RCLCPP_DEBUG(
                node_->get_logger(),
                "ActionExecutor Update: Executing next step of action, step_running_=%d", step_running_);

            action->Next();
            if (action->IsDone())
            {
                ActionFinished(action);
                action_.pop();
                if (action_.empty())
                {
                    action_done_ = true;
                } else
                {
                    Update();
                }
            }
        }
        else if (action != nullptr && action->IsDone())
        {
            RCLCPP_DEBUG(
                node_->get_logger(),
                "ActionExecutor Update: Action is done, step_running_=%d", step_running_);

            ActionFinished(action);

            action_.pop();
            if (action_.empty())
            {
                action_done_ = true;
            } else {
                Update();
            }
        }
    }

    void ActionExecutor::ReInit()
    {
        action_done_ = true;
        step_running_ = 0;

        std::queue<std::shared_ptr<BaseAction>> empty;
        std::swap(action_, empty);
    }

    void ActionExecutor::Down(BaseAction::Side front, bool force, bool inverted) {
        if (!arm_pos_[front].down || force)
        {
            RCLCPP_DEBUG(
                node_->get_logger(),
                "ActionExecutor Down called for front=%d, force=%d",
                static_cast<int>(front),
                static_cast<int>(force));

            auto action = std::make_shared<DownAction>(shared_from_this(), front, inverted);
            action_.push(action);
            if (action_done_)
            {
                step_running_ = 0;
            }
            action_done_ = false;

            Update();
        }
    }

    void ActionExecutor::Up(BaseAction::Side side, bool force) {
        if ((arm_pos_[side].down) || force)
        {
            RCLCPP_DEBUG(
                node_->get_logger(),
                "ActionExecutor Up called for side=%d, force=%d",
                static_cast<int>(side),
                static_cast<int>(force));

            auto action = std::make_shared<UPAction>(shared_from_this(), side);
            action_.push(action);
            if (action_done_)
            {
                step_running_ = 0;
            }
            action_done_ = false;

            Update();
        }
    }

    void ActionExecutor::ToggleArm(BaseAction::Side side, bool force)
    {
        RCLCPP_DEBUG(
            node_->get_logger(),
            "ActionExecutor ToggleArm called for side=%d, force=%d",
            static_cast<int>(side),
            static_cast<int>(force));

        if (side == BaseAction::BOTH)
        {
            ToggleArm(BaseAction::FRONT, force);
            ToggleArm(BaseAction::BACK, force);
            return;
        }

        if (arm_pos_[side].down)
        {
            Up(side, force);
        }
        else
        {
            Down(side, force, arm_pos_[side].rotated);
        }
    }

    void ActionExecutor::RotateArm(BaseAction::Side side, bool force, bool rotated)
    {
        if ((arm_pos_[side].rotated != rotated || !arm_pos_[side].down) || force)
        {
            RCLCPP_DEBUG(
                node_->get_logger(),
                "ActionExecutor RotateArm called for side=%d, force=%d, rotated=%d",
                static_cast<int>(side),
                static_cast<int>(force),
                static_cast<int>(rotated));

            if (arm_pos_[side].down)
            {
                auto action = std::make_shared<UPAction>(shared_from_this(), side);
                action_.push(action);
            }

            auto action = std::make_shared<DownAction>(shared_from_this(), side, rotated);
            action_.push(action);
            if (action_done_)
            {
                step_running_ = 0;
            }
            action_done_ = false;

            Update();
        }
    }

    void ActionExecutor::Take(const std::vector<std::pair<int, BaseAction::Side>>& servos) {
        RCLCPP_DEBUG(
            node_->get_logger(),
            "ActionExecutor Take called for %d servos",
            static_cast<int>(servos.size()));

        auto action = std::make_shared<TakeAction>(shared_from_this(), servos);
        action_.push(action);
        if (action_done_)
        {
            step_running_ = 0;
        }
        action_done_ = false;

        Update();
    }

    void ActionExecutor::Free(const std::vector<std::pair<int, BaseAction::Side>>& servos) {
        RCLCPP_DEBUG(
            node_->get_logger(),
            "ActionExecutor Free called for %d servos",
            static_cast<int>(servos.size()));

        auto action = std::make_shared<FreeAction>(shared_from_this(), servos);
        action_.push(action);
        if (action_done_)
        {
            step_running_ = 0;
        }
        action_done_ = false;

        Update();
    }

    void ActionExecutor::ToggleServo(const std::vector<std::pair<int, BaseAction::Side>>& servos)
    {
        RCLCPP_DEBUG(
            node_->get_logger(),
            "ActionExecutor ToggleServo called for %d servos",
            static_cast<int>(servos.size()));

        auto action  = std::make_shared<ToggleServoAction>(shared_from_this(), servos);
        action_.push(action);
        if (action_done_)
        {
            step_running_ = 0;
        }
        action_done_ = false;

        Update();
    }

    void ActionExecutor::MoveServoTimed(const modelec_interfaces::msg::ActionServoTimedArray& msg)
    {
        servo_timed_move_pub_->publish(msg);
        step_running_ += msg.items.size();

        RCLCPP_DEBUG(
            node_->get_logger(),
            "ActionExecutor MoveServoTimed called with %d items, step_running_=%d",
            static_cast<int>(msg.items.size()),
            step_running_);
    }

    void ActionExecutor::MoveServo(const modelec_interfaces::msg::ActionServoPosArray& msg)
    {
        servo_move_pub_->publish(msg);
        step_running_ += msg.items.size();
    }

    void ActionExecutor::ActivateThermo(BaseAction::Side side, bool deploy, bool force)
    {
        if (thermo_state_[side] != deploy || force)
        {
            auto action = std::make_shared<ThermoAction>(shared_from_this(), side, deploy);
            action_.push(action);
            if (action_done_)
            {
                step_running_ = 0;
            }
            action_done_ = false;

            Update();
        }
    }

    void ActionExecutor::LookOn(BaseAction::Side side, bool force)
    {
        if (cam_side_ != side || force)
        {
            auto action = std::make_shared<LookOnAction>(shared_from_this(), side);
            action_.push(action);
            if (action_done_)
            {
                step_running_ = 0;
            }
            action_done_ = false;

            Update();
        }
    }

    void ActionExecutor::ActionFinished(const std::shared_ptr<BaseAction>& action)
    {
        RCLCPP_DEBUG(
            node_->get_logger(),
            "ActionExecutor ActionFinished called for action");
        action->End();
    }

    bool ActionExecutor::IsEmpty() const
    {
        return box_obstacles_[0] == nullptr && box_obstacles_[1] == nullptr;
    }

    bool ActionExecutor::IsFull() const
    {
        return box_obstacles_[0] != nullptr && box_obstacles_[1] != nullptr;
    }

    bool ActionExecutor::HasBox(BaseAction::Side side) const
    {
        return box_obstacles_[side] != nullptr;
    }

    bool ActionExecutor::HasFrontBox() const
    {
        return HasBox(BaseAction::FRONT);
    }

    bool ActionExecutor::HasBackBox() const
    {
        return HasBox(BaseAction::BACK);
    }

    bool ActionExecutor::HasOneBox() const
    {
        return HasFrontBox() != HasBackBox();
    }

    void ActionExecutor::SendPoint(const int point) const
    {
        std_msgs::msg::Int64 msg;
        msg.data = point;
        score_pub_->publish(msg);
    }
}
