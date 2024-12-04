#pragma once

#include <rclcpp/rclcpp.hpp>
#include <unordered_set>
#include <wiringPi.h>
#include <modelec_interface/msg/solenoid.hpp>

namespace Modelec {
    class SolenoidController : public rclcpp::Node {
    public:
        SolenoidController();
    private:
        std::unordered_set<int> solenoid_pin_;

        rclcpp::Subscription<modelec_interface::msg::Solenoid>::SharedPtr solenoid_subscriber_;

        rclcpp::Subscription<modelec_interface::msg::Solenoid>::SharedPtr solenoid_add_subscriber_;

        void activateSolenoid(const modelec_interface::msg::Solenoid::SharedPtr msg);

        void addSolenoidPin(const modelec_interface::msg::Solenoid::SharedPtr msg);
    };
}
