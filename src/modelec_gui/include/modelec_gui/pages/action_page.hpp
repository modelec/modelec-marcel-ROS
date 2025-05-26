#pragma once

#include <QWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QSpinBox>

#include <modelec_gui/widget/action_widget.hpp>

#include <rclcpp/rclcpp.hpp>

#include <modelec_interfaces/msg/action_asc_pos.hpp>
#include <modelec_interfaces/msg/action_servo_pos.hpp>
#include <modelec_interfaces/msg/action_relay_state.hpp>
#include <modelec_interfaces/msg/action_exec.hpp>

namespace ModelecGUI
{
    class ActionPage : public QWidget
    {
        Q_OBJECT

    public:

        using ActionAscPos = modelec_interfaces::msg::ActionAscPos;
        using ActionServoPos = modelec_interfaces::msg::ActionServoPos;
        using ActionRelayState = modelec_interfaces::msg::ActionRelayState;
        using ActionExec = modelec_interfaces::msg::ActionExec;

        ActionPage(rclcpp::Node::SharedPtr node, QWidget* parent = nullptr);
        ~ActionPage() override;

        rclcpp::Node::SharedPtr get_node() const { return node_; }

    private:
        QPushButton* max_size_button_;

        QVBoxLayout* layout_;
        QHBoxLayout* asc_layout_;

        ActionWidget* asc_action_;
        bool waiting_for_move_asc_;

        QGridLayout* servo_layout_;

        ActionWidget* servo0_action_;
        ActionWidget* servo1_action_;
        ActionWidget* servo2_action_;
        ActionWidget* servo3_action_;
        ActionWidget* servo4_action_;
        ActionWidget* servo5_action_;
        ActionWidget* servo6_action_;
        ActionWidget* servo7_action_;
        ActionWidget* servo8_action_;
        std::vector<ActionWidget*> servo_actions_;
        std::vector<bool> waiting_for_move_servo_;

        QHBoxLayout* relay_layout_;

        QPushButton* relay_top_button_;
        QPushButton* relay_bottom_button_;
        QPushButton* relay_third_button_;
        std::vector<QPushButton*> relay_buttons_;
        std::vector<bool> relay_values_;

        QPushButton* deploy_banner_button_;

        rclcpp::Node::SharedPtr node_;

        rclcpp::Publisher<ActionExec>::SharedPtr action_exec_pub_;

        rclcpp::Subscription<ActionAscPos>::SharedPtr asc_get_sub_;
        rclcpp::Publisher<ActionAscPos>::SharedPtr asc_set_pub_;
        rclcpp::Subscription<ActionAscPos>::SharedPtr asc_set_res_sub_;
        rclcpp::Publisher<ActionAscPos>::SharedPtr asc_move_pub_;
        rclcpp::Subscription<ActionAscPos>::SharedPtr asc_move_res_sub_;

        rclcpp::Subscription<ActionServoPos>::SharedPtr servo_get_sub_;
        rclcpp::Publisher<ActionServoPos>::SharedPtr servo_set_pub_;
        rclcpp::Subscription<ActionServoPos>::SharedPtr servo_set_res_sub_;
        rclcpp::Publisher<ActionServoPos>::SharedPtr servo_move_pub_;
        rclcpp::Subscription<ActionServoPos>::SharedPtr servo_move_res_sub_;

        rclcpp::Subscription<ActionRelayState>::SharedPtr relay_get_sub_;
        rclcpp::Publisher<ActionRelayState>::SharedPtr relay_move_pub_;
        rclcpp::Subscription<ActionRelayState>::SharedPtr relay_move_res_sub_;
    };
} // namespace Modelec
