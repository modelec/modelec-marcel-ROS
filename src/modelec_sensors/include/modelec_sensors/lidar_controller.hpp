#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace Modelec {
    // TODO - DEPRECATED

    class LidarController : public rclcpp::Node {
    public:
        LidarController();
    private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

        void processLidarData(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    };
}