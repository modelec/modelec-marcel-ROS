#pragma once

#include <QLabel>
#include <QWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QSpinBox>
#include <QLabel>

#include <modelec_gui/widget/action_widget.hpp>

#include <rclcpp/rclcpp.hpp>

#include <modelec_interfaces/msg/action_asc_pos.hpp>
#include <modelec_interfaces/msg/action_servo_pos_array.hpp>
#include <modelec_interfaces/msg/action_servo_timed_array.hpp>
#include <modelec_interfaces/msg/action_relay_state_array.hpp>
#include <modelec_interfaces/msg/action_exec.hpp>

namespace ModelecGUI
{
    class ActionPage : public QWidget
    {
        Q_OBJECT

    public:

        using ActionAscPos = modelec_interfaces::msg::ActionAscPos;
        using ActionServoPosArray = modelec_interfaces::msg::ActionServoPosArray;
        using ActionRelayStateArray = modelec_interfaces::msg::ActionRelayStateArray;
        using ActionServoTimedArray = modelec_interfaces::msg::ActionServoTimedArray;
        using ActionExec = modelec_interfaces::msg::ActionExec;

        ActionPage(rclcpp::Node::SharedPtr node, QWidget* parent = nullptr);
        ~ActionPage() override;

        rclcpp::Node::SharedPtr get_node() const { return node_; }

    private:

        QVBoxLayout* main_layout_;
        QPushButton* deploy_max_button_;

        // ---- Action1 front ----
        QHBoxLayout* deploy_action1_layout_;
        QVBoxLayout* deploy_action1_front_layout_;
        QLabel* deploy_action1_front_label_;
        QPushButton* deploy_action1_front_up_button_;

        QHBoxLayout* deploy_action1_front_servos_layout_;

        QPushButton* deploy_action1_front_servo1_button_,
            *deploy_action1_front_servo2_button_,
            *deploy_action1_front_servo3_button_,
            *deploy_action1_front_servo4_button_;

        QPushButton* deploy_action1_front_down_button_;
        QPushButton* deploy_action1_front_rotate_button_;

        // ---- Action1 back ----
        QVBoxLayout* deploy_action1_back_layout_;
        QLabel* deploy_action1_back_label_;
        QPushButton* deploy_action1_back_up_button_;

        QHBoxLayout* deploy_action1_back_servos_layout_;
        QPushButton* deploy_action1_back_servo1_button_,
            *deploy_action1_back_servo2_button_,
            *deploy_action1_back_servo3_button_,
            *deploy_action1_back_servo4_button_;

        QPushButton* deploy_action1_back_down_button_;
        QPushButton* deploy_action1_back_rotate_button_;

        // ---- Action2 ----
        QVBoxLayout* deploy_action2_layout_;
        QLabel* deploy_action2_label_;
        QPushButton* deploy_action2_button_;
        QPushButton* undeploy_action2_button_;

        // ---- Ros ----
        rclcpp::Node::SharedPtr node_;

        rclcpp::Publisher<ActionExec>::SharedPtr action_exec_pub_;

        rclcpp::Subscription<ActionServoPosArray>::SharedPtr servo_get_sub_;
        rclcpp::Publisher<ActionServoPosArray>::SharedPtr servo_move_pub_;
        rclcpp::Subscription<ActionServoPosArray>::SharedPtr servo_move_res_sub_;

        rclcpp::Publisher<ActionServoTimedArray>::SharedPtr servo_timed_move_pub_;
        rclcpp::Subscription<ActionServoTimedArray>::SharedPtr servo_timed_move_res_sub_;

    };
} // namespace Modelec
