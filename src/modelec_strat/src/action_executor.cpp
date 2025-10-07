#include <modelec_strat/action_executor.hpp>

namespace Modelec
{
    ActionExecutor::ActionExecutor()
    {
    }

    ActionExecutor::ActionExecutor(const rclcpp::Node::SharedPtr& node) : node_(node)
    {
        asc_move_pub_ = node_->create_publisher<modelec_interfaces::msg::ActionAscPos>("/action/move/asc", 10);
        servo_move_pub_ = node_->create_publisher<modelec_interfaces::msg::ActionServoPos>("/action/move/servo", 10);
        relay_move_pub_ = node_->create_publisher<modelec_interfaces::msg::ActionRelayState>("/action/move/relay", 10);

        asc_move_res_sub_ = node_->create_subscription<modelec_interfaces::msg::ActionAscPos>(
            "/action/move/asc/res", 10, [this](const modelec_interfaces::msg::ActionAscPos::SharedPtr)
            {
                step_running_--;
                Update();
            });

        servo_move_res_sub_ = node_->create_subscription<modelec_interfaces::msg::ActionServoPos>(
            "/action/move/servo/res", 10, [this](const modelec_interfaces::msg::ActionServoPos::SharedPtr)
            {
                step_running_--;
                Update();
            });

        relay_move_res_sub_ = node_->create_subscription<modelec_interfaces::msg::ActionRelayState>(
            "/action/move/relay/res", 10, [this](const modelec_interfaces::msg::ActionRelayState::SharedPtr)
            {
                step_running_--;
                Update();
            });

        action_exec_sub_ = node_->create_subscription<modelec_interfaces::msg::ActionExec>(
            "/action/exec", 10, [this](const modelec_interfaces::msg::ActionExec::SharedPtr msg)
            {
                action_ = static_cast<Action>(msg->action);
                for (const auto& step : msg->mission)
                {
                    step_.push(static_cast<Step>(step));
                }
                action_done_ = false;
                step_running_ = 0;
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
        if (step_.empty())
        {
            action_ = NONE;
            action_done_ = true;
            return;
        }

        if (step_running_ <= 0)
        {
            switch (step_.front())
            {
            default:
                return;
            }

            step_.pop();
        }
    }

    void ActionExecutor::ReInit()
    {
        action_done_ = true;
        step_running_ = 0;
        while (!step_.empty())
        {
            step_.pop();
        }
        action_ = NONE;
    }
}
