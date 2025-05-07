#pragma once

#include <QWidget>
#include <QHBoxLayout>
#include <QPushButton>
#include <QSvgRenderer>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int8.hpp>

namespace ModelecGUI
{
    class HomePage : public QWidget
    {
        Q_OBJECT

    public:
        HomePage(rclcpp::Node::SharedPtr node, QWidget* parent = nullptr);

    public slots:
        void onYellowButtonClicked();

        void onBlueButtonClicked();

    signals:
        void TeamChoose();

    protected:
        void paintEvent(QPaintEvent*) override;

        rclcpp::Node::SharedPtr node_;

        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr team_publisher_;

        QVBoxLayout* v_layout_;
        QHBoxLayout* h_layout_;

        QPushButton* blue_button_;
        QPushButton* yellow_button_;

        QSvgRenderer* renderer_;

    };
}
