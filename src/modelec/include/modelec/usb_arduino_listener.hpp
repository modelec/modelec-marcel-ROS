#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <boost/asio.hpp>

#define SERIAL_PORT "/dev/ttyUSB0"
#define MAX_MESSAGE_LEN 1048
#define BAUDS 115200 //vitesse des données (bit/sec)

namespace Modelec {

class USBListener : public rclcpp::Node {
public:
    USBListener();
    ~USBListener() override;

private:
    int bauds = BAUDS;
    std::string serial_port = SERIAL_PORT;
    int max_message_len = MAX_MESSAGE_LEN;

    boost::asio::io_service io;
    boost::asio::serial_port port;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber;
    rclcpp::TimerBase::SharedPtr timer;

    void readData();

    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    rcl_interfaces::msg::SetParametersResult onParameterChange(const std::vector<rclcpp::Parameter> &parameters);
    void updateConnection();

    void write_to_arduino(const std::string &data);

    bool initializeConnection();
};

}