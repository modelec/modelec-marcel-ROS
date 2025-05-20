#pragma once

#include <QWidget>
#include <QHBoxLayout>
#include <QPushButton>
#include <QSvgRenderer>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/empty.hpp>

#include <modelec_interfaces/msg/spawn.hpp>

namespace ModelecGUI
{
    class HomePage : public QWidget
    {
    private:
        Q_OBJECT

    public:
        HomePage(rclcpp::Node::SharedPtr node, QWidget* parent = nullptr);

        void Init();

    signals:
        void TeamChoose();

    protected:
        void paintEvent(QPaintEvent*) override;

        rclcpp::Node::SharedPtr node_;

        rclcpp::Publisher<modelec_interfaces::msg::Spawn>::SharedPtr spawn_pub_;
        rclcpp::Subscription<modelec_interfaces::msg::Spawn>::SharedPtr spawn_sub_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reset_strat_pub_;
        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr ask_spawn_client_;

        QSvgRenderer* renderer_;

        std::vector<QPushButton*> spawn_buttons_;
    };
}
