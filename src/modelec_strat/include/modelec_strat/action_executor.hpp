#pragma once

#include <rclcpp/rclcpp.hpp>
#include <modelec_interfaces/msg/action_asc_pos.hpp>
#include <modelec_interfaces/msg/action_relay_state_array.hpp>
#include <modelec_interfaces/msg/action_servo_pos_array.hpp>
#include <modelec_interfaces/msg/action_servo_timed_array.hpp>
#include <modelec_interfaces/msg/action_exec.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

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

        void Down(BaseAction::Side side, bool force = false, bool inverted = false);

        void Up(BaseAction::Side side, bool force = false);

        void ToggleArm(BaseAction::Side side, bool force = false);

        void RotateArm(BaseAction::Side side, bool force = false, bool rotated = false);

        void Take(const std::vector<std::pair<int, BaseAction::Side>>& servos);

        void Free(const std::vector<std::pair<int, BaseAction::Side>>& servos);

        void ToggleServo(const std::vector<std::pair<int, BaseAction::Side>>& servos);

        void MoveServoTimed(const modelec_interfaces::msg::ActionServoTimedArray& msg);

        void MoveServo(const modelec_interfaces::msg::ActionServoPosArray& msg);

        void ActivateThermo(BaseAction::Side side, bool deploy, bool force = false);

        void LookOn(BaseAction::Side side, bool force = false);

        void ActionFinished(const std::shared_ptr<BaseAction>& action);

        std::array<std::shared_ptr<BoxObstacle>, 2> box_obstacles_;

        std::array<bool, 8> servo_pos_;

        struct ArmState
        {
            bool down;
            bool rotated;
        };
        std::array<ArmState, 2> arm_pos_;

        std::array<float, 16> servo_value_ = {
            2.91,
            0.95,
            3.2,
            0,
            1,
            1,
            1,
            1,
        };

        std::map<BaseAction::Side, bool> thermo_state_ = {
            {BaseAction::LEFT, false},
            {BaseAction::RIGHT, false},
        };

        BaseAction::Side cam_side_ = BaseAction::Side::CENTER;

        bool looking_on_front_ = true;

        bool IsEmpty() const;

        bool IsFull() const;

        bool HasBox(BaseAction::Side side) const;

        bool HasFrontBox() const;

        bool HasBackBox() const;

        bool HasOneBox() const;

        void SendPoint(int point) const;

        void AskColor() const;

    protected:
        rclcpp::Publisher<modelec_interfaces::msg::ActionAscPos>::SharedPtr asc_move_pub_;
        rclcpp::Publisher<modelec_interfaces::msg::ActionServoPosArray>::SharedPtr servo_move_pub_;
        rclcpp::Publisher<modelec_interfaces::msg::ActionRelayStateArray>::SharedPtr relay_move_pub_;
        rclcpp::Publisher<modelec_interfaces::msg::ActionServoTimedArray>::SharedPtr servo_timed_move_pub_;

        rclcpp::Subscription<modelec_interfaces::msg::ActionAscPos>::SharedPtr asc_move_res_sub_;
        rclcpp::Subscription<modelec_interfaces::msg::ActionServoPosArray>::SharedPtr servo_move_res_sub_;
        rclcpp::Subscription<modelec_interfaces::msg::ActionRelayStateArray>::SharedPtr relay_move_res_sub_;
        rclcpp::Subscription<modelec_interfaces::msg::ActionServoTimedArray>::SharedPtr servo_timed_move_res_sub_;

        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr ask_color_client_;

        rclcpp::Subscription<modelec_interfaces::msg::ActionExec>::SharedPtr action_exec_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

        rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr score_pub_;

        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr ask_pub_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr color_sub_;

        std::queue<std::shared_ptr<BaseAction>> action_;

        bool action_done_ = true;
        int step_running_ = 0;

        bool servo_step_by_step_;

        int step_value_;

        int goal_value_;

        int current_step_;

        float last_left_trig = 1.0f;
        float last_right_trig = 1.0f;

        bool last_l3 = false;
        bool last_r3 = false;

    private:
        rclcpp::Node::SharedPtr node_;
    };
}
