#pragma once

#include <rclcpp/rclcpp.hpp>
#include <unordered_set>
#include <wiringPi.h>
#include <modelec_interface/msg/solenoid.hpp>
#include <modelec_interface/srv/add_solenoid.hpp>

namespace Modelec {
    class SolenoidController : public rclcpp::Node {
    public:
        SolenoidController();
    private:
        std::unordered_set<int> solenoid_pin_;

        rclcpp::Subscription<modelec_interface::msg::Solenoid>::SharedPtr solenoid_subscriber_;

        rclcpp::Service<modelec_interface::srv::AddSolenoid>::SharedPtr add_solenoid_service_;

        void activateSolenoid(const modelec_interface::msg::Solenoid::SharedPtr msg);

        void addSolenoid(const std::shared_ptr<modelec_interface::srv::AddSolenoid::Request> request, std::shared_ptr<modelec_interface::srv::AddSolenoid::Response> response);
    };
}
