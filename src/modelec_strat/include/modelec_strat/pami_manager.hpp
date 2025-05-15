#pragma once

#include <rclcpp/rclcpp.hpp>

#include <modelec_utils/config.hpp>
#include <modelec_utils/point.hpp>

#include <modelec_strat/obstacle/obstacle.hpp>

#include <tinyxml2.h>

#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/empty.hpp>

namespace Modelec
{
    class PamiManger : public rclcpp::Node
    {
    public:
        PamiManger();

        bool ReadFromXML(const std::string& filename);

    protected:
        std::vector<Obstacle> zones_;

        int time_to_put_zone_ = 0;
        int time_to_remove_top_pot_ = 0;

        int score_to_add_ = 0;

        int score_goupie_ = 0;
        int score_superstar_ = 0;
        int score_all_party_ = 0;
        int score_free_zone_ = 0;

        rclcpp::TimerBase::SharedPtr timer_add_;
        rclcpp::TimerBase::SharedPtr timer_remove_;

        rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr start_time_sub_;
        rclcpp::Publisher<modelec_interfaces::msg::Obstacle>::SharedPtr add_obs_pub_;
        rclcpp::Publisher<modelec_interfaces::msg::Obstacle>::SharedPtr remove_obs_pub_;
        rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr score_pub_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_strat_sub_;
    };
}
