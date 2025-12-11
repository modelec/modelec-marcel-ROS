#include <modelec_strat/action_executor.hpp>

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
                step_running_--;
                Update();
            });

        relay_move_res_sub_ = node_->create_subscription<modelec_interfaces::msg::ActionRelayStateArray>(
            "/action/move/relay/res", 10, [this](const modelec_interfaces::msg::ActionRelayStateArray::SharedPtr)
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
            case SEND_STEP:
                {
                    modelec_interfaces::msg::ActionServoPosArray msg;

                    msg.items[0].id = 0;
                    msg.items[0].angle = 0;

                    msg.items[1].id = 1;
                    msg.items[1].angle = M_PI_2;

                    servo_move_pub_->publish(msg);

                    step_running_ = 1;
                }
                break;
            case TAKE_STEP:
                {
                    modelec_interfaces::msg::ActionServoPosArray msg;

                    msg.items[0].id = 0;
                    msg.items[0].angle = M_PI_2;

                    msg.items[1].id = 1;
                    msg.items[1].angle = 0;

                    servo_move_pub_->publish(msg);

                    step_running_ = 1;
                }
                break;
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

    void ActionExecutor::Send() {
        if (action_done_)
        {
            action_ = SEND;
            action_done_ = false;
            step_running_ = 0;

            step_.push(SEND_STEP);

            Update();
        }
    }

    void ActionExecutor::Take() {
        if (action_done_)
        {
            action_ = TAKE;
            action_done_ = false;
            step_running_ = 0;

            step_.push(TAKE_STEP);

            Update();
        }
    }
}
