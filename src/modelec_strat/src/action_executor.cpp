#include <modelec_strat/action_executor.hpp>

#include "modelec_strat/action/down_action.hpp"
#include "modelec_strat/action/up_action.hpp"
#include "modelec_strat/action/free_action.hpp"
#include "modelec_strat/action/take_action.hpp"
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
            "/action/move/servo/res", 10, [this](const modelec_interfaces::msg::ActionServoPosArray::SharedPtr)
            {
                // BUG
                // if ServoTimed is called this one will trigger so step_running_ will be decremented at the beginning of the Timed one
                // step_running_ -= msg->items.size();
                // Update();
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

                if (token.size() == 0)
                {
                    RCLCPP_WARN(
                        node_->get_logger(),
                        "Empty action received");
                    return;
                }

                action_ = BaseAction::CreateAction(
                    token[0],
                    shared_from_this());
                if (action_ == nullptr)
                {
                    RCLCPP_WARN(
                        node_->get_logger(),
                        "Unknown action: %s",
                        msg->action.c_str());
                    return;
                }
                action_->Init(token);
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
                if (msg->buttons.size() >= 5)
                {
                    if (msg->buttons[0] == 1) // A button
                    {
                        Down(BaseAction::BOTH);
                    }
                    else if (msg->buttons[1] == 1) // B button
                    {
                        Up(BaseAction::BOTH);
                    }
                    else if (msg->buttons[3] == 1) // X button
                    {
                        Take({{0, BaseAction::FRONT}, {1, BaseAction::FRONT}, {2, BaseAction::FRONT}, {3, BaseAction::FRONT}});
                    }
                    else if (msg->buttons[4] == 1) // Y button
                    {
                        Free({{0, BaseAction::FRONT}, {1, BaseAction::FRONT}, {2, BaseAction::FRONT}, {3, BaseAction::FRONT}});
                    }
                }
            });
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
        if (action_ != nullptr && !action_->IsDone() && step_running_ <= 0)
        {
            RCLCPP_DEBUG(
                node_->get_logger(),
                "ActionExecutor Update: Executing next step of action, step_running_=%d", step_running_);

            action_->Next();
            if (action_->IsDone())
            {
                action_done_ = true;
                action_ = nullptr;
            }
        }
        else if (action_ != nullptr && action_->IsDone())
        {
            RCLCPP_DEBUG(
                node_->get_logger(),
                "ActionExecutor Update: Action is done, step_running_=%d", step_running_);

            action_done_ = true;
            action_ = nullptr;
        }
    }

    void ActionExecutor::ReInit()
    {
        action_done_ = true;
        step_running_ = 0;
        action_ = nullptr;
    }

    void ActionExecutor::Down(BaseAction::Front front, bool force) {
        if (action_done_ || force)
        {
            action_ = std::make_shared<DownAction>(shared_from_this(), front);
            if (action_done_)
            {
                step_running_ = 0;
            }
            action_done_ = false;

            Update();
        }
    }

    void ActionExecutor::Up(BaseAction::Front front, bool force) {
        if (action_done_ || force)
        {
            action_ = std::make_shared<UPAction>(shared_from_this(), front);
            if (action_done_)
            {
                step_running_ = 0;
            }
            action_done_ = false;

            Update();
        }
    }

    void ActionExecutor::Take(std::vector<std::pair<int, BaseAction::Front>> servos, bool force) {
        if (action_done_ || force)
        {
            action_ = std::make_shared<TakeAction>(shared_from_this(), servos);
            if (action_done_)
            {
                step_running_ = 0;
            }
            action_done_ = false;

            Update();
        }
    }

    void ActionExecutor::Free(std::vector<std::pair<int, BaseAction::Front>> servos, bool force) {
        if (action_done_ || force)
        {
            action_ = std::make_shared<FreeAction>(shared_from_this(), servos);
            if (action_done_)
            {
                step_running_ = 0;
            }
            action_done_ = false;

            Update();
        }
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

    bool ActionExecutor::IsEmpty() const
    {
        return box_obstacles_[0] == nullptr && box_obstacles_[1] == nullptr;
    }

    bool ActionExecutor::HasBox(BaseAction::Front front) const
    {
        return box_obstacles_[front] != nullptr;
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
}
