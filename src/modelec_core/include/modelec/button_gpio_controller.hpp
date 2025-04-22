#pragma once

#include <rclcpp/rclcpp.hpp>
#include <modelec_interfaces/srv/add_button.hpp>
#include <modelec_interfaces/srv/button.hpp>
#include <modelec_interfaces/msg/button.hpp>
#include <wiringPi.h>

namespace Modelec {
    class ButtonGpioController : public rclcpp::Node {

        struct Button {
            int pin;
            rclcpp::Publisher<modelec_interfaces::msg::Button>::SharedPtr publisher;
            std::string name;
        };

    public:
        ButtonGpioController();

    private:
        // service
        rclcpp::Service<modelec_interfaces::srv::AddButton>::SharedPtr new_button_service_;
        rclcpp::Service<modelec_interfaces::srv::Button>::SharedPtr button_server_;

        // service callbacks
        void new_button(const std::shared_ptr<modelec_interfaces::srv::AddButton::Request> request, std::shared_ptr<modelec_interfaces::srv::AddButton::Response> response);
        void check_button(const std::shared_ptr<modelec_interfaces::srv::Button::Request> request, std::shared_ptr<modelec_interfaces::srv::Button::Response> response);

        // timer
        rclcpp::TimerBase::SharedPtr timer_;

        std::map<int, Button> buttons_;
    };
}