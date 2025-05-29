#include <modelec_strat/action_executor.hpp>

namespace Modelec
{
    ActionExecutor::ActionExecutor()
    {
    }

    ActionExecutor::ActionExecutor(const rclcpp::Node::SharedPtr& node) : node_(node)
    {
        /*asc_get_pub_ = node_->create_publisher<modelec_interfaces::msg::ActionAscPos>("/action/get/asc", 10);
        servo_get_pub_ = node_->create_publisher<modelec_interfaces::msg::ActionServoPos>("/action/get/servo", 10);
        relay_get_pub_ = node_->create_publisher<modelec_interfaces::msg::ActionRelayState>("/action/get/relay", 10);

        asc_get_res_sub_ = node_->create_subscription<modelec_interfaces::msg::ActionAscPos>(
            "/action/get/asc/res", 10, [this](const modelec_interfaces::msg::ActionAscPos::SharedPtr)
            {
            });

        servo_get_res_sub_ = node_->create_subscription<modelec_interfaces::msg::ActionServoPos>(
            "/action/get/servo/res", 10, [this](const modelec_interfaces::msg::ActionServoPos::SharedPtr)
            {
            });

        relay_get_res_sub_ = node_->create_subscription<modelec_interfaces::msg::ActionRelayState>(
            "/action/get/relay/res", 10, [this](const modelec_interfaces::msg::ActionRelayState::SharedPtr)
            {
            });

        asc_set_pub_ = node_->create_publisher<modelec_interfaces::msg::ActionAscPos>("/action/set/asc", 10);
        servo_set_pub_ = node_->create_publisher<modelec_interfaces::msg::ActionServoPos>("/action/set/servo", 10);

        asc_set_res_sub_ = node_->create_subscription<modelec_interfaces::msg::ActionAscPos>(
            "/action/set/asc/res", 10, [this](const modelec_interfaces::msg::ActionAscPos::SharedPtr)
            {
            });

        servo_set_res_sub_ = node_->create_subscription<modelec_interfaces::msg::ActionServoPos>(
            "/action/set/servo/res", 10, [this](const modelec_interfaces::msg::ActionServoPos::SharedPtr)
            {
            });*/

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
            case DEPLOY_BANNER_STEP:
                {
                    modelec_interfaces::msg::ActionServoPos msg;
                    msg.id = 5;
                    msg.pos = 0;
                    servo_move_pub_->publish(msg);

                    step_running_ = 1;
                }

                break;

            case UNDEPLOY_BANNER_STEP:
                {
                    modelec_interfaces::msg::ActionServoPos msg;
                    msg.id = 5;
                    msg.pos = 1;
                    servo_move_pub_->publish(msg);

                    step_running_ = 1;
                }

                break;

            case ASC_GO_DOWN:
                {
                    modelec_interfaces::msg::ActionAscPos asc_msg;
                    asc_msg.pos = 1;
                    asc_move_pub_->publish(asc_msg);

                    modelec_interfaces::msg::ActionServoPos servo_action_bottom_msg;
                    servo_action_bottom_msg.id = 4;
                    servo_action_bottom_msg.pos = 1;
                    servo_move_pub_->publish(servo_action_bottom_msg);

                    step_running_ = 2;
                }

                break;
            case STICK_TO_STRUCT:
                {
                    modelec_interfaces::msg::ActionRelayState relay_top_msg;
                    relay_top_msg.state = true;
                    relay_top_msg.id = 2;
                    relay_move_pub_->publish(relay_top_msg);

                    modelec_interfaces::msg::ActionRelayState relay_bottom_msg;
                    relay_bottom_msg.state = true;
                    relay_bottom_msg.id = 1;
                    relay_move_pub_->publish(relay_bottom_msg);

                    modelec_interfaces::msg::ActionServoPos first_pot_msg;
                    first_pot_msg.id = 0;
                    first_pot_msg.pos = 0;
                    servo_move_pub_->publish(first_pot_msg);

                    modelec_interfaces::msg::ActionServoPos fourth_pot_msg;
                    fourth_pot_msg.id = 1;
                    fourth_pot_msg.pos = 0;
                    servo_move_pub_->publish(fourth_pot_msg);

                    step_running_ = 4;
                }

                break;
            case ASC_GO_UP:
                {
                    modelec_interfaces::msg::ActionAscPos asc_msg;
                    asc_msg.pos = 3;
                    asc_move_pub_->publish(asc_msg);

                    step_running_ = 1;
                }

                break;
            case RETRACT_BOTTOM_PLATE:
                {
                    modelec_interfaces::msg::ActionServoPos servo_action_bottom_msg;
                    servo_action_bottom_msg.id = 2;
                    servo_action_bottom_msg.pos = 2;
                    servo_move_pub_->publish(servo_action_bottom_msg);

                    step_running_ = 1;
                }

                break;
            case ASC_GO_DOWN_TO_TAKE_POT:
                {
                    modelec_interfaces::msg::ActionAscPos asc_msg;
                    asc_msg.pos = 0;
                    asc_move_pub_->publish(asc_msg);

                    step_running_ = 1;
                }

                break;
            case STICK_POT:
                {
                    modelec_interfaces::msg::ActionServoPos top_pot_msg;
                    top_pot_msg.id = 4;
                    top_pot_msg.pos = 0;
                    servo_move_pub_->publish(top_pot_msg);

                    step_running_ = 1;
                }

                break;
            case ASC_GO_UP_TO_TAKE_POT:
                {
                    modelec_interfaces::msg::ActionAscPos asc_msg;
                    asc_msg.pos = 3;
                    asc_move_pub_->publish(asc_msg);

                    modelec_interfaces::msg::ActionServoPos top_pot_msg;
                    top_pot_msg.id = 4;
                    top_pot_msg.pos = 1;
                    servo_move_pub_->publish(top_pot_msg);

                    step_running_ = 2;
                }

                break;
            case PLACE_FIRST_PLATE:
                {
                    modelec_interfaces::msg::ActionServoPos action_bottom_msg;
                    action_bottom_msg.id = 2;
                    action_bottom_msg.pos = 0;
                    servo_move_pub_->publish(action_bottom_msg);

                    step_running_ = 1;
                }

                break;
            case STICK_ALL:
                {
                    modelec_interfaces::msg::ActionServoPos top_pot_msg;
                    top_pot_msg.id = 4;
                    top_pot_msg.pos = 0;
                    servo_move_pub_->publish(top_pot_msg);

                    step_running_ = 1;
                }

                break;
            case ASC_FINAL:
                {
                    modelec_interfaces::msg::ActionAscPos asc_msg;
                    asc_msg.pos = 2;
                    asc_move_pub_->publish(asc_msg);

                    step_running_ = 1;
                }

                break;
            case FREE_ALL:
                {
                    modelec_interfaces::msg::ActionRelayState relay_top_msg;
                    relay_top_msg.state = false;
                    relay_top_msg.id = 2;
                    relay_move_pub_->publish(relay_top_msg);

                    modelec_interfaces::msg::ActionRelayState relay_bottom_msg;
                    relay_bottom_msg.state = false;
                    relay_bottom_msg.id = 1;
                    relay_move_pub_->publish(relay_bottom_msg);

                    modelec_interfaces::msg::ActionServoPos first_pot_msg;
                    first_pot_msg.id = 0;
                    first_pot_msg.pos = 1;
                    servo_move_pub_->publish(first_pot_msg);

                    modelec_interfaces::msg::ActionServoPos second_pot_msg;
                    second_pot_msg.id = 1;
                    second_pot_msg.pos = 1;
                    servo_move_pub_->publish(second_pot_msg);

                    modelec_interfaces::msg::ActionServoPos top_pot_msg;
                    top_pot_msg.id = 4;
                    top_pot_msg.pos = 0;
                    servo_move_pub_->publish(top_pot_msg);

                    step_running_ = 5;
                }

                break;
            case REMOVE_ACTION_STEP:
                {
                    modelec_interfaces::msg::ActionAscPos asc_msg;
                    asc_msg.pos = 3;
                    asc_move_pub_->publish(asc_msg);

                    modelec_interfaces::msg::ActionServoPos servo_action_bottom_msg;
                    servo_action_bottom_msg.id = 3;
                    servo_action_bottom_msg.pos = 0;
                    servo_move_pub_->publish(servo_action_bottom_msg);

                    step_running_ = 2;
                }

                break;
            default:
                return;
            }

            step_.pop();
        }
    }

    void ActionExecutor::DeployBanner()
    {
        if (action_done_)
        {
            action_ = DEPLOY_BANNER;
            action_done_ = false;
            step_running_ = 0;

            step_.push(DEPLOY_BANNER_STEP);

            Update();
        }
    }

    void ActionExecutor::UndeployBanner()
    {
        if (action_done_)
        {
            action_ = UNDEPLOY_BANNER;
            action_done_ = false;
            step_running_ = 0;

            step_.push(UNDEPLOY_BANNER_STEP);

            Update();
        }
    }

    void ActionExecutor::TakePot(bool two_floor)
    {
        if (action_done_)
        {
            action_ = TAKE_POT;
            action_done_ = false;
            step_running_ = 0;

            if (two_floor)
            {
                step_.push(ASC_GO_DOWN);
                step_.push(STICK_TO_STRUCT);
                step_.push(ASC_GO_UP);
                step_.push(RETRACT_BOTTOM_PLATE);
                step_.push(ASC_GO_DOWN_TO_TAKE_POT);
                step_.push(STICK_POT);
                step_.push(ASC_GO_UP_TO_TAKE_POT);
                step_.push(PLACE_FIRST_PLATE);
            }
            else
            {
                step_.push(ASC_GO_DOWN);
                step_.push(STICK_ALL);
            }

            Update();
        }
    }

    void ActionExecutor::PlacePot(bool two_floor)
    {
        if (action_done_)
        {
            action_ = PLACE_POT;
            action_done_ = false;
            step_running_ = 0;

            if (two_floor)
            {
                step_.push(ASC_FINAL);
                step_.push(FREE_ALL);
                step_.push(REMOVE_ACTION_STEP);
            }
            else
            {
                step_.push(FREE_ALL);
                step_.push(REMOVE_ACTION_STEP);
            }

            Update();
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
