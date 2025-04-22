#pragma once

#include <rclcpp/rclcpp.hpp>
#include <modelec_interfaces/srv/tirette.hpp>
#include <std_msgs/msg/bool.hpp>
#include <wiringPi.h>

#define REFRESH_RATE 100
#define GPIO_TIRETTE 17

using TiretteInterface = modelec_interfaces::srv::Tirette;

namespace Modelec {
    class TiretteController : public rclcpp::Node {
    public:
        TiretteController();

    private:
        rclcpp::Service<TiretteInterface>::SharedPtr service;
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher;
        bool tirette_state;

        void check_tirette(const std::shared_ptr<TiretteInterface::Request> request, std::shared_ptr<TiretteInterface::Response> response);
    };
}
