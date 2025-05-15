#pragma once

#include <rclcpp/rclcpp.hpp>
#include <modelec_interfaces/msg/action_asc_pos.hpp>
#include <modelec_interfaces/msg/action_relay_state.hpp>
#include <modelec_interfaces/msg/action_servo_pos.hpp>

namespace Modelec
{

    class ActionExecutor
    {
    public:

        enum Action
        {
            NONE,
            DEPLOY_BANNER,
            TAKE_POT,
            PLACE_POT,
        };

        enum Step
        {
        };

        ActionExecutor();

        ActionExecutor(const rclcpp::Node::SharedPtr& node);

        rclcpp::Node::SharedPtr GetNode() const;

        bool IsActionDone() const;

        void Update();

        void DeployBanner();

        void TakePot();

        void PlacePot();

    protected:
        /*
        rclcpp::Publisher<modelec_interfaces::msg::ActionAscPos>::SharedPtr asc_get_pub_;
        rclcpp::Publisher<modelec_interfaces::msg::ActionServoPos>::SharedPtr servo_get_pub_;
        rclcpp::Publisher<modelec_interfaces::msg::ActionRelayState>::SharedPtr relay_get_pub_;

        rclcpp::Subscription<modelec_interfaces::msg::ActionAscPos>::SharedPtr asc_get_res_sub_;
        rclcpp::Subscription<modelec_interfaces::msg::ActionServoPos>::SharedPtr servo_get_res_sub_;
        rclcpp::Subscription<modelec_interfaces::msg::ActionRelayState>::SharedPtr relay_get_res_sub_;

        rclcpp::Publisher<modelec_interfaces::msg::ActionAscPos>::SharedPtr asc_set_pub_;
        rclcpp::Publisher<modelec_interfaces::msg::ActionServoPos>::SharedPtr servo_set_pub_;

        rclcpp::Subscription<modelec_interfaces::msg::ActionAscPos>::SharedPtr asc_set_res_sub_;
        rclcpp::Subscription<modelec_interfaces::msg::ActionServoPos>::SharedPtr servo_set_res_sub_;
        */

        rclcpp::Publisher<modelec_interfaces::msg::ActionAscPos>::SharedPtr asc_move_pub_;
        rclcpp::Publisher<modelec_interfaces::msg::ActionServoPos>::SharedPtr servo_move_pub_;
        rclcpp::Publisher<modelec_interfaces::msg::ActionRelayState>::SharedPtr relay_move_pub_;

        rclcpp::Subscription<modelec_interfaces::msg::ActionAscPos>::SharedPtr asc_move_res_sub_;
        rclcpp::Subscription<modelec_interfaces::msg::ActionServoPos>::SharedPtr servo_move_res_sub_;
        rclcpp::Subscription<modelec_interfaces::msg::ActionRelayState>::SharedPtr relay_move_res_sub_;

        Action action_ = NONE;

        std::queue<Step> step_;

        bool action_done_ = true;
        bool step_done_ = true;

    private:
        rclcpp::Node::SharedPtr node_;
    };

}