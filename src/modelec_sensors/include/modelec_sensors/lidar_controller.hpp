#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace Modelec {
    class LidarController : public rclcpp::Node {
    private:

    public:
        LidarController();
    private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

        void processLidarData(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    };
}