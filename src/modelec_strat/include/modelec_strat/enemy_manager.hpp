#pragma once

#include <rclcpp/rclcpp.hpp>

#include <modelec_interfaces/msg/odometry_pos.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace Modelec
{
    class EnemyManager : public rclcpp::Node
    {
    public:
        EnemyManager();

    protected:
        void OnCurrentPos(const modelec_interfaces::msg::OdometryPos::SharedPtr msg);

        void OnLidarData(const sensor_msgs::msg::LaserScan::SharedPtr msg);

        void TimerCallback();

    private:

        rclcpp::Subscription<modelec_interfaces::msg::OdometryPos>::SharedPtr current_pos_sub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
        rclcpp::Publisher<modelec_interfaces::msg::OdometryPos>::SharedPtr enemy_pos_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        modelec_interfaces::msg::OdometryPos current_pos_;

        modelec_interfaces::msg::OdometryPos last_enemy_pos_;
        bool enemy_initialized_ = false;
        rclcpp::Time last_publish_time_;
    };
}
