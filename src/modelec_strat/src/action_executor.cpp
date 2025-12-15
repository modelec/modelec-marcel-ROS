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
                action_ = static_cast<Action>(msg->action);
                for (const auto& step : msg->mission)
                {
                    step_.push(static_cast<Step>(step));
                }
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
            case FRONT_DOWN_STEP:
                {
                    modelec_interfaces::msg::ActionServoTimedArray msg;
                    msg.items.resize(4);

                    msg.items[0].id = 0;
                    msg.items[0].start_angle = 1.49;
                    msg.items[0].end_angle = 0;
                    msg.items[0].duration_s = 1;

                    msg.items[1].id = 1;
                    msg.items[1].start_angle = 1.5;
                    msg.items[1].end_angle = 3;
                    msg.items[1].duration_s = 1;

                    msg.items[2].id = 4;
                    msg.items[2].start_angle = 3;
                    msg.items[2].end_angle = 1.45;
                    msg.items[2].duration_s = 1;

                    msg.items[3].id = 5;
                    msg.items[3].start_angle = 0;
                    msg.items[3].end_angle = 1.6;
                    msg.items[3].duration_s = 1;

                    servo_timed_move_pub_->publish(msg);

                    step_running_ = msg.items.size();
                }
                break;
            case FRONT_UP_STEP:
                {
                    modelec_interfaces::msg::ActionServoTimedArray msg;
                    msg.items.resize(4);

                    msg.items[0].id = 0;
                    msg.items[0].start_angle = 0;
                    msg.items[0].end_angle = 1.49;
                    msg.items[0].duration_s = 1;

                    msg.items[1].id = 1;
                    msg.items[1].start_angle = 3;
                    msg.items[1].end_angle = 1.5;
                    msg.items[1].duration_s = 1;

                    msg.items[2].id = 4;
                    msg.items[2].start_angle = 1.45;
                    msg.items[2].end_angle = 3;
                    msg.items[2].duration_s = 1;

                    msg.items[3].id = 5;
                    msg.items[3].start_angle = 1.6;
                    msg.items[3].end_angle = 0;
                    msg.items[3].duration_s = 1;

                    servo_timed_move_pub_->publish(msg);

                    step_running_ = msg.items.size();
                }
                break;
            case FRONT_TAKE_1_STEP:
                {
                    modelec_interfaces::msg::ActionServoPosArray msg;

                    msg.items[0].id = 2;
                    msg.items[0].angle = 0;

                    servo_move_pub_->publish(msg);

                    step_running_ = msg.items.size();
                }
                break;
            case FRONT_FREE_1_STEP:
                {
                    modelec_interfaces::msg::ActionServoPosArray msg;

                    msg.items[0].id = 2;
                    msg.items[0].angle = M_PI;

                    servo_move_pub_->publish(msg);

                    step_running_ = msg.items.size();
                }

                break;
            case MAX_DEPLOY_STEP:
                {

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

    void ActionExecutor::Down(bool front) {
        if (action_done_)
        {
            action_ = front ? FRONT_DOWN : BACK_DOWN;
            action_done_ = false;
            step_running_ = 0;

            step_.push(front ? FRONT_DOWN_STEP : BACK_DOWN_STEP);

            Update();
        }
    }

    void ActionExecutor::Up(bool front) {
        if (action_done_)
        {
            action_ = front ? FRONT_UP : BACK_UP;
            action_done_ = false;
            step_running_ = 0;

            step_.push(front ? FRONT_UP_STEP : BACK_UP_STEP);

            Update();
        }
    }

    void ActionExecutor::Take(bool front, int n) {
        if (action_done_)
        {
            action_ = static_cast<Action>(n * 10 + (front ? 0 : 100));
            action_done_ = false;
            step_running_ = 0;

            step_.push(static_cast<Step>(n * 10 + (front ? 0 : 100)));

            Update();
        }
    }

    void ActionExecutor::Free(bool front, int n) {
        if (action_done_)
        {
            action_ = static_cast<Action>(1 + n * 10 + (front ? 0 : 100));
            action_done_ = false;
            step_running_ = 0;

            step_.push(static_cast<Step>(1 + n * 10 + (front ? 0 : 100)));

            Update();
        }
    }
}
