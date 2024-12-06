#pragma once

#include <modelec_interface/msg/pca9685_servo.hpp>
#include <array>
#include <rclcpp/rclcpp.hpp>
#include <modelec_interface/msg/pca9685_servo.hpp>
#include <modelec_interface/msg/servo_mode.hpp>
#include <modelec_interface/srv/new_servo_motor.hpp>
#include <rclcpp/subscription.hpp>
#include <map>

#define PINCE_1_PIN 0
#define PINCE_2_PIN 1
#define PINCE_3_PIN 2
#define ARM_1_PIN 3
#define ARM_2_PIN 4

namespace Modelec {
    class ArmController : public rclcpp::Node {

        using Mode = modelec_interface::msg::ServoMode;

        struct Pince {
            int pin;
            int mode;
            // angle mapping : 1 - PINCE_CLOSED | 2 - PINCE_MIDDLE | 3 - PINCE_OPEN | 4 - PINCE_FULLY_OPEN
            std::array<int, 4> angles;
        };

        struct Arm {
            std::map<int, std::array<int, 3>> pins;
            int mode;
        };

    public:
        ArmController();
    private:
        rclcpp::Publisher<modelec_interface::msg::PCA9685Servo>::SharedPtr publisher_;
        rclcpp::Subscription<modelec_interface::msg::ServoMode>::SharedPtr subscription_;

        std::unordered_map<int, Pince> pince_pins = {
            {PINCE_1_PIN, {PINCE_1_PIN, Mode::PINCE_CLOSED, {136, 123, 112, 102}}},
            {PINCE_2_PIN, {PINCE_2_PIN, Mode::PINCE_CLOSED, {42, 32, 19, 15}}},
            {PINCE_3_PIN, {PINCE_3_PIN, Mode::PINCE_CLOSED, {152, 141, 125, 122}}},
        };

        Arm arm = {
            {
                {ARM_1_PIN, {180, 168, 52}},
                {ARM_2_PIN, {0, 12, 128}},
            },
            Mode::ARM_BOTTOM
        };

        // service lient to add a servo
        rclcpp::Client<modelec_interface::srv::NewServoMotor>::SharedPtr client_;

        void ControlPince(const Mode::SharedPtr msg);
        void ControlArm(const Mode::SharedPtr msg);
    };
}