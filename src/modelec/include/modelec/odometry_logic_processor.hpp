#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <modelec/utils.hpp>
#include <modelec_interface/msg/odometry_data.hpp>

namespace Modelec {
  class odometry_logic_processor : public rclcpp::Node {
  public:
    odometry_logic_processor();
  private:
    void processData(const std::string &data);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_to_odometry_;
    rclcpp::Publisher<modelec_interface::msg::OdometryData>::SharedPtr publisher_odometry_;
  };
}
