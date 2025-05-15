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
                step_done_ = true;
                Update();
            });

        servo_move_res_sub_ = node_->create_subscription<modelec_interfaces::msg::ActionServoPos>(
            "/action/move/servo/res", 10, [this](const modelec_interfaces::msg::ActionServoPos::SharedPtr)
            {
                step_done_ = true;
                Update();
            });

        relay_move_res_sub_ = node_->create_subscription<modelec_interfaces::msg::ActionRelayState>(
            "/action/move/relay/res", 10, [this](const modelec_interfaces::msg::ActionRelayState::SharedPtr)
            {
                step_done_ = true;
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

        step_.pop();
        step_done_ = false;
    }

    void ActionExecutor::DeployBanner()
    {
        action_ = DEPLOY_BANNER;
        action_done_ = false;

        Update();
    }

    void ActionExecutor::TakePot()
    {
        action_ = TAKE_POT;
        action_done_ = false;

        Update();
    }

    void ActionExecutor::PlacePot()
    {
        action_ = PLACE_POT;
        action_done_ = false;

        Update();
    }
}
