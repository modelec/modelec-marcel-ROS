#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <modelec/utils.hpp>
#include <modelec_interface/msg/arduino_data.hpp>

namespace Modelec {
  class LogicProcessor : public rclcpp::Node {
  public:
    LogicProcessor();
  private:
    void processData(const std::string &data);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    rclcpp::Publisher<modelec_interface::msg::ArduinoData>::SharedPtr publisher_;
  };
}