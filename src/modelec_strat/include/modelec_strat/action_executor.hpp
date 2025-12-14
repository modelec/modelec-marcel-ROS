#pragma once

#include <rclcpp/rclcpp.hpp>
#include <modelec_interfaces/msg/action_asc_pos.hpp>
#include <modelec_interfaces/msg/action_relay_state_array.hpp>
#include <modelec_interfaces/msg/action_servo_pos_array.hpp>
#include <modelec_interfaces/msg/action_servo_timed_array.hpp>
#include <modelec_interfaces/msg/action_exec.hpp>

namespace Modelec
{
    class ActionExecutor
    {
    public:
        enum Action
        {
            NONE = 0,

            FRONT_UP = 1,
            FRONT_DOWN = 2,

            FRONT_TAKE_1 = 10,
            FRONT_FREE_1 = 11,
            FRONT_TAKE_2 = 20,
            FRONT_FREE_2 = 21,
            FRONT_TAKE_3 = 30,
            FRONT_FREE_3 = 31,
            FRONT_TAKE_4 = 40,
            FRONT_FREE_4 = 41,

            BACK_UP = 101,
            BACK_DOWN = 102,

            BACK_TAKE_1 = 110,
            BACK_FREE_1 = 111,
            BACK_TAKE_2 = 120,
            BACK_FREE_2 = 121,
            BACK_TAKE_3 = 130,
            BACK_FREE_3 = 131,
            BACK_TAKE_4 = 140,
            BACK_FREE_4 = 141,

            THERMO_DEPLOY = 200,
            THERMO_UNDEPLOY = 201,

            MAX_DEPLOY = 99
        };

        enum Step
        {
            FRONT_UP_STEP = 1,
            FRONT_DOWN_STEP = 2,

            FRONT_TAKE_1_STEP = 10,
            FRONT_FREE_1_STEP = 11,
            FRONT_TAKE_2_STEP = 20,
            FRONT_FREE_2_STEP = 21,
            FRONT_TAKE_3_STEP = 30,
            FRONT_FREE_3_STEP = 31,
            FRONT_TAKE_4_STEP = 40,
            FRONT_FREE_4_STEP = 41,

            BACK_UP_STEP = 101,
            BACK_DOWN_STEP = 102,

            BACK_TAKE_1_STEP = 110,
            BACK_FREE_1_STEP = 111,
            BACK_TAKE_2_STEP = 120,
            BACK_FREE_2_STEP = 121,
            BACK_TAKE_3_STEP = 130,
            BACK_FREE_3_STEP = 131,
            BACK_TAKE_4_STEP = 140,
            BACK_FREE_4_STEP = 141,

            THERMO_DEPLOY_STEP = 200,
            THERMO_UNDEPLOY_STEP = 201,

            MAX_DEPLOY_STEP = 99,
        };

        ActionExecutor();

        ActionExecutor(const rclcpp::Node::SharedPtr& node);

        rclcpp::Node::SharedPtr GetNode() const;

        bool IsActionDone() const;

        void Update();

        void ReInit();

        void Down(bool front = true);

        void Up(bool front = true);

        void Take(bool front = true, int n = 1);

        void Free(bool front = true, int n = 1);

    protected:
        rclcpp::Publisher<modelec_interfaces::msg::ActionAscPos>::SharedPtr asc_move_pub_;
        rclcpp::Publisher<modelec_interfaces::msg::ActionServoPosArray>::SharedPtr servo_move_pub_;
        rclcpp::Publisher<modelec_interfaces::msg::ActionRelayStateArray>::SharedPtr relay_move_pub_;
        rclcpp::Publisher<modelec_interfaces::msg::ActionServoTimedArray>::SharedPtr servo_timed_move_pub_;

        rclcpp::Subscription<modelec_interfaces::msg::ActionAscPos>::SharedPtr asc_move_res_sub_;
        rclcpp::Subscription<modelec_interfaces::msg::ActionServoPosArray>::SharedPtr servo_move_res_sub_;
        rclcpp::Subscription<modelec_interfaces::msg::ActionRelayStateArray>::SharedPtr relay_move_res_sub_;
        rclcpp::Subscription<modelec_interfaces::msg::ActionServoTimedArray>::SharedPtr servo_timed_move_res_sub_;

        rclcpp::Subscription<modelec_interfaces::msg::ActionExec>::SharedPtr action_exec_sub_;

        Action action_ = NONE;

        std::queue<Step> step_;

        bool action_done_ = true;
        int step_running_ = 0;

        bool servo_step_by_step_;

        int step_value_;

        int goal_value_;

        int current_step_;

    private:
        rclcpp::Node::SharedPtr node_;
    };
}
