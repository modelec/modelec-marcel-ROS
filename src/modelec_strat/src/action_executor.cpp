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
            "/action/move/servo/res", 10, [this](const modelec_interfaces::msg::ActionServoPosArray::SharedPtr msg)
            {
                step_running_ -= msg->items.size();
                Update();
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
            action_->Next();
            if (action_->IsDone())
            {
                action_done_ = true;
                action_ = nullptr;
            }
        }

        if (action_ != nullptr && action_->IsDone())
        {
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

    void ActionExecutor::Down(bool front) {
        if (action_done_)
        {
            action_ = std::make_shared<DownAction>(shared_from_this(), front);
            action_done_ = false;
            step_running_ = 0;

            Update();
        }
    }

    void ActionExecutor::Up(bool front) {
        if (action_done_)
        {
            action_ = std::make_shared<UPAction>(shared_from_this(), front);
            action_done_ = false;
            step_running_ = 0;

            Update();
        }
    }

    void ActionExecutor::Take(bool front, int n) {
        if (action_done_)
        {
            action_ = std::make_shared<TakeAction>(shared_from_this(), front, n);
            action_done_ = false;
            step_running_ = 0;

            Update();
        }
    }

    void ActionExecutor::Free(bool front, int n) {
        if (action_done_)
        {
            action_ = std::make_shared<FreeAction>(shared_from_this(), front, n);
            action_done_ = false;
            step_running_ = 0;

            Update();
        }
    }

    void ActionExecutor::MoveServoTimed(const modelec_interfaces::msg::ActionServoTimedArray& msg)
    {
        servo_timed_move_pub_->publish(msg);
        step_running_ += msg.items.size();
    }

    void ActionExecutor::MoveServo(const modelec_interfaces::msg::ActionServoPosArray& msg)
    {
        servo_move_pub_->publish(msg);
        step_running_ += msg.items.size();
    }
}
