#pragma once

#include <rclcpp/rclcpp.hpp>

#include <modelec_interfaces/msg/odometry_pos.hpp>
#include <modelec_interfaces/msg/strat_state.hpp>
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
        rclcpp::Subscription<modelec_interfaces::msg::StratState>::SharedPtr state_sub_;
        rclcpp::Publisher<modelec_interfaces::msg::OdometryPos>::SharedPtr enemy_pos_pub_;
        rclcpp::Publisher<modelec_interfaces::msg::OdometryPos>::SharedPtr close_enemy_pos_pub_;
        rclcpp::Publisher<modelec_interfaces::msg::OdometryPos>::SharedPtr enemy_long_time_pub_;

        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Time last_movement_time_;

        modelec_interfaces::msg::OdometryPos current_pos_;

        modelec_interfaces::msg::OdometryPos last_enemy_pos_;
        bool enemy_initialized_ = false;
        rclcpp::Time last_publish_time_;

        bool is_enemy_close_ = false;

        float min_move_threshold_mm_ = 0.0f;
        float refresh_rate_s_ = 0.0f;

        bool game_stated_ = false;
        float max_stationary_time_s_ = 10.0f;
        float map_width_ = 0;
        float map_height_ = 0;
        float margin_detection_table_ = 0.0f;
        float robot_width_ = 0;
        float robot_length_ = 0;
        double robot_radius_ = 0;
        float min_emergency_distance_ = 0.0f;
    };
}
