#include "modelec/usb_aruido_logic_processor.hpp"
#include <vector>

using namespace Modelec;

LogicProcessor::LogicProcessor() : Node("usb_logic_processor") {
    subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "arduino_raw_data", 10, [this](std_msgs::msg::String::SharedPtr msg) {
            processData(msg->data);
        });
    publisher_ = this->create_publisher<modelec_interface::msg::ArduinoData>("arduino_processed_data", 10);
}

void LogicProcessor::processData(const std::string &data) {
  auto processed_data = "Processed: " + data;
  auto msg = modelec_interface::msg::ArduinoData();
  std::vector<std::string> d = split(data, ':');

  if (d.size() == 3) {
    msg.x = std::stof(d[0]);
    msg.y = std::stof(d[1]);
    msg.theta = std::stof(d[2]);
  }
  else {
    msg.x = 0;
    msg.y = 0;
    msg.theta = 0;
  }

  publisher_->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Modelec::LogicProcessor>());
  rclcpp::shutdown();
  return 0;
}