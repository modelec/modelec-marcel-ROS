#pragma once

#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>
#include <std_msgs/msg/string.hpp>
#include <modelec_interface/srv/add_serial_listener.hpp>
#include <modelec_interface/msg/write_serial.hpp>
#include <deque>
#include <thread>
#include <mutex>

#define MAX_MESSAGE_LEN 1048

namespace Modelec
{
class SerialListener {
private:
    bool status_;
    int bauds_;
    std::string serial_port_;
    int max_message_len_;

    boost::asio::io_service& io_;
    std::vector<char> read_buffer_;
    std::deque<std::string> write_queue_;
    std::mutex write_mutex_;
    std::thread io_thread_;

    void start_async_read();
    void start_async_write();

public:
    std::string name_;
    boost::asio::serial_port port_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;

    SerialListener(const std::string& name, int bauds, const std::string& serial_port,
                   int max_message_len, boost::asio::io_service& io);

    ~SerialListener();

    void SetMaxMessageLen(int max_message_len) { max_message_len_ = max_message_len; }
    bool IsOk() const { return status_; }

    void write(std_msgs::msg::String::SharedPtr msg);
};

class MultipleSerialListener : public rclcpp::Node
{
public:
    MultipleSerialListener();
    ~MultipleSerialListener() override;

private:
    int default_max_message_len_ = MAX_MESSAGE_LEN;

    std::map<std::string, std::shared_ptr<SerialListener>> serial_listeners;

    boost::asio::io_service io;

    rclcpp::Service<modelec_interface::srv::AddSerialListener>::SharedPtr add_serial_listener_service_;
    void add_serial_listener(
        const std::shared_ptr<modelec_interface::srv::AddSerialListener::Request> request,
        std::shared_ptr<modelec_interface::srv::AddSerialListener::Response> response);

    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    rcl_interfaces::msg::SetParametersResult onParameterChange(const std::vector<rclcpp::Parameter> &parameters);
    void updateConnection();
};
}  // namespace Modelec
