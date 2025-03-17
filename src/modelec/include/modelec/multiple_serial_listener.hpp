#pragma once

#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>
#include <std_msgs/msg/string.hpp>
#include <modelec_interface/srv/add_serial_listener.hpp>
#include <thread>

#define MAX_MESSAGE_LEN 1048
#define READ_REFRESH_RATE 100 //ms

namespace Modelec
{
class MultipleSerialListener : public rclcpp::Node
{
public:
    MultipleSerialListener();
    ~MultipleSerialListener() override;

private:

    int default_max_message_len_ = MAX_MESSAGE_LEN;
    int read_refresh_rate_ = READ_REFRESH_RATE;

    class SerialListener {
    private:
        bool status_;
        int bauds_;
        std::string serial_port_;
        int max_message_len_;
        boost::asio::streambuf read_buf_;
        boost::asio::io_service& io_;

    public:
        std::string name_;
        boost::asio::serial_port port_;

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;

        SerialListener(const std::string& name, int bauds, const std::string& serial_port, int max_message_len, boost::asio::io_service& io);

        void SetMaxMessageLen(int max_message_len) { max_message_len_ = max_message_len; }
        bool IsOk() const { return status_; }
        void read();
        void write(const std::string& data);
    };

    std::map<std::string, std::shared_ptr<SerialListener>> serial_listeners;

    boost::asio::io_service io;
    rclcpp::TimerBase::SharedPtr timer;

    rclcpp::Service<modelec_interface::srv::AddSerialListener>::SharedPtr add_serial_listener_service_;
    void add_serial_listener(const std::shared_ptr<modelec_interface::srv::AddSerialListener::Request> request, std::shared_ptr<modelec_interface::srv::AddSerialListener::Response> response);

    void readData(const SerialListener& listener);

    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    rcl_interfaces::msg::SetParametersResult onParameterChange(const std::vector<rclcpp::Parameter> &parameters);
    void updateConnection();
};
}  // namespace Modelec
