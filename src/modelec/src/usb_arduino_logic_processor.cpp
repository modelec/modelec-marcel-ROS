#include "modelec/usb_aruido_logic_processor.hpp"
#include "modelec_interface/srv/add_serial_listener.hpp"
#include <vector>
#include <modelec_interface/srv/add_serial_listener.hpp>

using namespace Modelec;

LogicProcessor::LogicProcessor() : Node("usb_logic_processor") {

  // Service to create a new serial listener
  auto request = std::make_shared<modelec_interface::srv::AddSerialListener::Request>();
  request->name = "arduino";
  request->bauds = 115200;
  request->serial_port = "/dev/ttyACM0";
  auto client = this->create_client<modelec_interface::srv::AddSerialListener>("add_serial_listener");
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
    if (!result.get()->status) {
      RCLCPP_ERROR(this->get_logger(), "Failed to add serial listener");
    } else {
      RCLCPP_INFO(this->get_logger(), "Added serial listener");
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Service call failed");
  }

    subscriber_ = this->create_subscription<std_msgs::msg::String>(
        result.get()->publisher, 10, [this](std_msgs::msg::String::SharedPtr msg) {
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