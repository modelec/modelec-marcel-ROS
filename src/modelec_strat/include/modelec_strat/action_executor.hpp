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
            DEPLOY_BANNER,
            TAKE_POT,
            PLACE_POT,
            DEPLOY_MAX_SIZE,
        };

        enum Step
        {
            // Banner
            DEPLOY_BANNER_STEP,

            // Take Pot
            ASC_GO_DOWN,
            STICK_TO_STRUCT,
            ASC_GO_UP,
            RETRACT_BOTTOM_PLATE,
            ASC_GO_DOWN_TO_TAKE_POT,
            STICK_POT,
            ASC_GO_UP_TO_TAKE_POT,
            PLACE_FIRST_PLATE,

            // Take One floor
            STICK_ALL,


            // Place Pot
            ASC_FINAL,
            FREE_ALL,
            REMOVE_ACTION_STEP,
        };

        ActionExecutor();

        ActionExecutor(const rclcpp::Node::SharedPtr& node);

        rclcpp::Node::SharedPtr GetNode() const;

        bool IsActionDone() const;

        void Update();

        void DeployBanner();

        void TakePot(bool two_floor = true);

        void PlacePot(bool two_floor = true);

        void ReInit();

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

        rclcpp::Subscription<modelec_interfaces::msg::ActionExec>::SharedPtr action_exec_sub_;

        Action action_ = NONE;

        std::queue<Step> step_;

        bool action_done_ = true;
        int step_running_ = 0;

    private:
        rclcpp::Node::SharedPtr node_;
    };
}
