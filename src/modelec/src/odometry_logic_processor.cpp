#include "modelec/odometry_logic_processor.hpp"
#include "modelec_interface/srv/add_serial_listener.hpp"
#include <vector>

Modelec::odometry_logic_processor::odometry_logic_processor() : Node("usb_logic_processor") {
  publisher_odometry_ = this->create_publisher<modelec_interface::msg::OdometryData>("odomertry_data", 10);

  // Service to create a new serial listener
  auto request = std::make_shared<modelec_interface::srv::AddSerialListener::Request>();
  request->name = "odometry";
  request->bauds = 115200;
  request->serial_port = "/dev/pts/6";
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
    if (auto res = result.get()) {
      if (res->success) {

        RCLCPP_INFO(this->get_logger(), "Publisher: %s", res->publisher.c_str());
        RCLCPP_INFO(this->get_logger(), "Subscriber: %s", res->subscriber.c_str());

        subscriber_ = this->create_subscription<std_msgs::msg::String>(
          res->publisher, 10, [this](std_msgs::msg::String::SharedPtr msg) {
              processData(msg->data);
            });
      
        publisher_to_odometry_ = this->create_publisher<std_msgs::msg::String>(res->subscriber, 10);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to add serial listener");
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to ask for a serial listener");
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Service call failed");
  }
}

void Modelec::odometry_logic_processor::processData(const std::string &data) {
  auto processed_data = "Processed: " + data;
  RCLCPP_INFO(this->get_logger(), "%s", processed_data.c_str());
  auto msg = modelec_interface::msg::OdometryData();
  std::vector<std::string> d = split(data, ':');

  if (d.size() == 3) {
    msg.x = std::stol(d[0]);
    msg.y = std::stol(d[1]);
    msg.theta = std::stol(d[2]);
  }
  else {
    msg.x = 0;
    msg.y = 0;
    msg.theta = 0;
  }

  publisher_odometry_->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Modelec::odometry_logic_processor>());
  rclcpp::shutdown();
  return 0;
}