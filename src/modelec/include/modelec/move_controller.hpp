#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#define REFRESH_RATE 100

namespace Modelec {
    class MoveController : public rclcpp::Node {
    public:
        MoveController();

    private:
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber;

        float speedX, speedZ;

        float x, y, z, theta;
        float x_target, y_target, z_target, theta_target;

        void move();

        void move_target_callback(const std_msgs::msg::String::SharedPtr msg);
    };
}