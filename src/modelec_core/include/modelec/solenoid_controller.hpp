#pragma once

#include <rclcpp/rclcpp.hpp>
#include <unordered_set>
#include <wiringPi.h>
#include <modelec_interfaces/msg/solenoid.hpp>
#include <modelec_interfaces/srv/add_solenoid.hpp>

namespace Modelec {
    class SolenoidController : public rclcpp::Node {
    public:
        SolenoidController();
    private:
        std::unordered_set<int> solenoid_pin_;

        rclcpp::Subscription<modelec_interfaces::msg::Solenoid>::SharedPtr solenoid_subscriber_;

        rclcpp::Service<modelec_interfaces::srv::AddSolenoid>::SharedPtr add_solenoid_service_;

        void activateSolenoid(const modelec_interfaces::msg::Solenoid::SharedPtr msg);

        void addSolenoid(const std::shared_ptr<modelec_interfaces::srv::AddSolenoid::Request> request, std::shared_ptr<modelec_interfaces::srv::AddSolenoid::Response> response);
    };
}
