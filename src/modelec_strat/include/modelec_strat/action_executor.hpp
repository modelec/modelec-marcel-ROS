#pragma once

#include <rclcpp/rclcpp.hpp>
#include <modelec_interfaces/msg/action_asc_pos.hpp>
#include <modelec_interfaces/msg/action_relay_state.hpp>
#include <modelec_interfaces/msg/action_servo_pos.hpp>
#include <modelec_interfaces/msg/action_exec.hpp>

namespace Modelec
{
    class ActionExecutor
    {
    public:
        enum Action
        {
            NONE,
            TAKE,
            SEND,
        };

        enum Step
        {
            TAKE_STEP,
            SEND_STEP
        };

        ActionExecutor();

        ActionExecutor(const rclcpp::Node::SharedPtr& node);

        rclcpp::Node::SharedPtr GetNode() const;

        bool IsActionDone() const;

        void Update();

        void ReInit();

        void Send();

        void Take();

    protected:
        rclcpp::Publisher<modelec_interfaces::msg::ActionAscPos>::SharedPtr asc_move_pub_;
        rclcpp::Publisher<modelec_interfaces::msg::ActionServoPos>::SharedPtr servo_move_pub_;
        rclcpp::Publisher<modelec_interfaces::msg::ActionRelayState>::SharedPtr relay_move_pub_;

        rclcpp::Subscription<modelec_interfaces::msg::ActionAscPos>::SharedPtr asc_move_res_sub_;
        rclcpp::Subscription<modelec_interfaces::msg::ActionServoPos>::SharedPtr servo_move_res_sub_;
        rclcpp::Subscription<modelec_interfaces::msg::ActionRelayState>::SharedPtr relay_move_res_sub_;

        rclcpp::Subscription<modelec_interfaces::msg::ActionExec>::SharedPtr action_exec_sub_;

        Action action_ = NONE;

        std::queue<Step> step_;

        bool action_done_ = true;
        int step_running_ = 0;

    private:
        rclcpp::Node::SharedPtr node_;
    };
}
