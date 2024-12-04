#include "modelec/usb_arduino_listener.hpp"

namespace Modelec {
  USBListener::USBListener() : Node("usb_listener"), io(), port(io) {
    
    // Declare and initialize parameters
    bauds = this->declare_parameter<int>("bauds", BAUDS);
    serial_port = this->declare_parameter<std::string>("serial_port", SERIAL_PORT);
    max_message_len = this->declare_parameter<int>("max_message_len", MAX_MESSAGE_LEN);

    // Initialize topic publisher
    publisher = this->create_publisher<std_msgs::msg::String>("arduino_raw_data", 10);

    subscriber = this->create_subscription<std_msgs::msg::String>(
      "send_to_arduino", 10, [this](std_msgs::msg::String::SharedPtr msg) {
        write_to_arduino(msg->data);
      });

    // Open usb port
    if (!initializeConnection()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize the serial port, shutting down.");
      rclcpp::shutdown();
    }

    // Start reading periodically
    timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&USBListener::readData, this));

    // Set a callback for parameter updates
    parameter_callback_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &parameters) {
        return onParameterChange(parameters);
      });
  }

  bool USBListener::initializeConnection() {
    try {
      // Try to open the serial port
      port.open(serial_port);
      port.set_option(boost::asio::serial_port_base::baud_rate(bauds));
      RCLCPP_INFO(this->get_logger(), "Serial port '%s' opened successfully", serial_port.c_str());
      return true;
    } catch (boost::system::system_error &e) {
      RCLCPP_ERROR(this->get_logger(), "Error opening serial port '%s': %s", serial_port.c_str(), e.what());
      return false;
    }
  }

  void USBListener::readData() {
    if (!port.is_open()) {
      return;  // Port is not open, so exit
    }

    std::vector<char> data(max_message_len);
    try {
      // Attempt to read data from the serial port
      size_t len = port.read_some(boost::asio::buffer(data.data(), max_message_len));
      if (len > 0) {
        // Prepare and publish the message
        auto msg = std_msgs::msg::String();
        msg.data = std::string(data.begin(), data.begin() + len);
        publisher->publish(msg);
      }
    } catch (boost::system::system_error &e) {
      RCLCPP_ERROR(this->get_logger(), "Error reading from serial port: %s", e.what());
    }
  }

  USBListener::~USBListener() {
    // Cleanup resources and close port
    parameter_callback_handle_.reset();
    if (port.is_open()) {
      port.close();
    }
  }

  rcl_interfaces::msg::SetParametersResult USBListener::onParameterChange(const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &parameter : parameters) {
      if (parameter.get_name() == "bauds" || parameter.get_name() == "serial_port" || parameter.get_name() == "max_message_len") {
        updateConnection();
      }
    }

    return result;
  }

  void USBListener::updateConnection() {
    // Retrieve the current parameters
    int new_bauds = this->get_parameter("bauds").as_int();
    std::string new_serial_port = this->get_parameter("serial_port").as_string();
    int new_max_message_len = this->get_parameter("max_message_len").as_int();

    // Check if the parameters have changed
    if (new_bauds == bauds && new_serial_port == serial_port && new_max_message_len == max_message_len) {
      RCLCPP_DEBUG(this->get_logger(), "Connection parameters have not changed.");
      return;  // No changes, no need to update the connection
    }

    // Update the connection parameters
    bauds = new_bauds;
    serial_port = new_serial_port;
    max_message_len = new_max_message_len;

    // Reopen the connection with updated parameters
    try {
      if (port.is_open()) {
        port.close();  // Close the current port before reopening
      }

      if (!initializeConnection()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to reopen the serial port, shutting down.");
        rclcpp::shutdown();  // Graceful shutdown if port opening fails
      }

    } catch (boost::system::system_error &e) {
      RCLCPP_ERROR(this->get_logger(), "Error during connection update: %s", e.what());
      rclcpp::shutdown();
    }
  }

  void USBListener::write_to_arduino(const std::string &data) {
    try {
      boost::asio::write(port, boost::asio::buffer(data));
    } catch (boost::system::system_error &e) {
      RCLCPP_ERROR(this->get_logger(), "Error writing to serial port: %s", e.what());
    }
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Modelec::USBListener>());
  rclcpp::shutdown();
  return 0;
}
