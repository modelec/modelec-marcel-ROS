#pragma once

#include <rclcpp/rclcpp.hpp>
#include <modelec_interfaces/msg/action_asc_pos.hpp>
#include <modelec_interfaces/msg/action_relay_state_array.hpp>
#include <modelec_interfaces/msg/action_servo_pos_array.hpp>
#include <modelec_interfaces/msg/action_servo_timed_array.hpp>
#include <modelec_interfaces/msg/action_exec.hpp>

#include <sensor_msgs/msg/joy.hpp>

#include "action/base_action.hpp"
#include "obstacle/box.hpp"

namespace Modelec
{
    class BaseAction;

    class ActionExecutor : public std::enable_shared_from_this<ActionExecutor>
    {
    public:
        ActionExecutor();

        ActionExecutor(const rclcpp::Node::SharedPtr& node);

        rclcpp::Node::SharedPtr GetNode() const;

        bool IsActionDone() const;

        void Update();

        void ReInit();

        void Down(BaseAction::Front front, bool force = false);

        void Up(BaseAction::Front front, bool force = false);

        void ToggleArm(BaseAction::Front front, bool force = false);

        void RotateArm(BaseAction::Front front, bool force = false, bool rotated = false);

        void Take(const std::vector<std::pair<int, BaseAction::Front>>& servos, bool force = false);

        void Free(const std::vector<std::pair<int, BaseAction::Front>>& servos, bool force = false);

        void ToggleServo(const std::vector<std::pair<int, BaseAction::Front>>& servos, bool force = false);

        void MoveServoTimed(const modelec_interfaces::msg::ActionServoTimedArray& msg);

        void MoveServo(const modelec_interfaces::msg::ActionServoPosArray& msg);

        std::array<std::shared_ptr<BoxObstacle>, 2> box_obstacles_;

        std::array<bool, 8> servo_pos_;

        struct ArmState
        {
            bool down;
            bool rotated;
        };
        std::array<ArmState, 2> arm_pos_;

        bool IsEmpty() const;

        bool IsFull() const;

        bool HasBox(BaseAction::Front front) const;

        bool HasFrontBox() const;

        bool HasBackBox() const;

        bool HasOneBox() const;

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
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

        std::shared_ptr<BaseAction> action_;

        bool action_done_ = true;
        int step_running_ = 0;

        bool servo_step_by_step_;

        int step_value_;

        int goal_value_;

        int current_step_;

        float last_left_trig = 1.0f;
        float last_right_trig = 1.0f;

    private:
        rclcpp::Node::SharedPtr node_;
    };
}
