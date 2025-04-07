#pragma once

#include <rclcpp/rclcpp.hpp>
#include <modelec_interface/msg/odometry_speed.hpp>
#include <fstream>

namespace Modelec
{
    class SpeedResultNode : public rclcpp::Node {
    public:
        SpeedResultNode();
        ~SpeedResultNode() override;

    private:

        rclcpp::Time start_time_;

        std::string fileName_;

        std::ofstream file_;

        rclcpp::Subscription<modelec_interface::msg::OdometrySpeed>::SharedPtr sub_speed_;

        void SpeedCallback(const modelec_interface::msg::OdometrySpeed::SharedPtr msg);
    };
}