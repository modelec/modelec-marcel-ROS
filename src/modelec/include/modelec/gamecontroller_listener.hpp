#pragma once

#include "modelec_interface/msg/pca9685_servo.hpp"
#include <array>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <modelec_interface/msg/servo_mode.hpp>
#include <modelec_interface/msg/pca9685_servo.hpp>
#include <modelec_interface/srv/new_servo_motor.hpp>
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

        struct SolarPannelServo {
            int pin;
            float startAngle;
            float endAngle;
        };
        
        std::array<SolarPannelServo, 2> solarPannelServos = {
            { { 6, 16.0f, 40.0f }, { 7, 25.0f, 3.0f } }
        };

    public:
        ControllerListener();

    private:
        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
        rclcpp::Publisher<ServoMode>::SharedPtr servo_publisher_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arduino_publisher_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr clear_pca_publisher_;
        rclcpp::Publisher<modelec_interface::msg::PCA9685Servo>::SharedPtr pca9685_publisher_;
        rclcpp::Client<modelec_interface::srv::NewServoMotor>::SharedPtr client_;

        void CheckButton(const sensor_msgs::msg::Joy::SharedPtr msg);
        void CheckAxis(const sensor_msgs::msg::Joy::SharedPtr msg);

        bool button_2_was_pressed = false;
        bool button_3_was_pressed = false;
        bool button_1_was_pressed = false;
        bool button_0_was_pressed = false;
        bool button_4_was_pressed = false;
        bool button_5_was_pressed = false;

        int last_speed = 0;
        int last_rotation = 0;
        float last_solar_1_angle = 0;
        float last_solar_2_angle = 0;
    };
}
