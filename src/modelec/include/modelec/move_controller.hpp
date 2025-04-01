#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#define REFRESH_RATE 100

namespace Modelec {
    class MoveController : public rclcpp::Node {
    public:
        MoveController();

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

        float x = 0, y = 0, theta = 0;

        void PublishPosition();
    };
}