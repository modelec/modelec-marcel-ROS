#pragma once

#include <QWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QSpinBox>

#include <rclcpp/rclcpp.hpp>

#include <modelec_interfaces/msg/odometry_pos.hpp>
#include <modelec_interfaces/msg/odometry_add_waypoint.hpp>
#include <modelec_interfaces/srv/odometry_speed.hpp>
#include <modelec_interfaces/srv/odometry_start.hpp>
#include <modelec_interfaces/srv/odometry_get_pid.hpp>
#include <modelec_interfaces/srv/odometry_set_pid.hpp>


namespace ModelecGUI
{
    class OdoPage : public QWidget
    {
        Q_OBJECT

    public:
        OdoPage(rclcpp::Node::SharedPtr node, QWidget* parent = nullptr);
        ~OdoPage() override;

        rclcpp::Node::SharedPtr get_node() const { return node_; }

    private:
        QVBoxLayout* mainLayout_;
        QPushButton* startButton_;
        QLineEdit *xBox_, *yBox_, *thetaBox_;
        QHBoxLayout* posLayout_;

        QPushButton* askPID_;
        QDoubleSpinBox *pPIDBox_, *iPIDBox_, *dPIDBox_;
        QHBoxLayout* pidLayout_;
        QPushButton* setPID_;

        QPushButton* askSpeed_;
        QLineEdit *xSpeedBox_, *ySpeedBox_, *thetaSpeedBox_;
        QHBoxLayout* speedLayout_;

        rclcpp::Node::SharedPtr node_;

        rclcpp::Subscription<modelec_interfaces::msg::OdometryPos>::SharedPtr sub_;

        // client
        rclcpp::Client<modelec_interfaces::srv::OdometrySpeed>::SharedPtr client_;
        rclcpp::Client<modelec_interfaces::srv::OdometryStart>::SharedPtr client_start_;
        rclcpp::Client<modelec_interfaces::srv::OdometryGetPid>::SharedPtr client_get_pid_;
        rclcpp::Client<modelec_interfaces::srv::OdometrySetPid>::SharedPtr client_set_pid_;

        void PositionCallback(const modelec_interfaces::msg::OdometryPos::SharedPtr msg);
    };
} // namespace Modelec
