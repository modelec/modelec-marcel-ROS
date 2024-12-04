#pragma once

#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <modelec_interface/msg/servo_mode.hpp>
#include <modelec/utils.hpp>

namespace Modelec {
    class ControllerListener : public rclcpp::Node
    {
        using ServoMode = modelec_interface::msg::ServoMode;

        std::array<int, 3> pinces = {
            ServoMode::PINCE_CLOSED,
            ServoMode::PINCE_CLOSED,
            ServoMode::PINCE_CLOSED,
        };

        int arm = ServoMode::ARM_BOTTOM;
    public:
        ControllerListener();

    private:
        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
        rclcpp::Publisher<ServoMode>::SharedPtr servo_publisher_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arduino_publisher_;

        void CheckButton(const sensor_msgs::msg::Joy::SharedPtr msg);
        void CheckAxis(const sensor_msgs::msg::Joy::SharedPtr msg);

        bool button_2_was_pressed = false;
        bool button_3_was_pressed = false;
        bool button_1_was_pressed = false;
        bool button_0_was_pressed = false;
        bool button_9_was_pressed = false;
        bool button_8_was_pressed = false;

        int last_speed = 0;
        int last_rotation = 0;
    };
}
