#pragma once

#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>
#include <std_msgs/msg/string.hpp>
#include <modelec_interfaces/srv/add_serial_listener.hpp>
#include <deque>
#include <thread>
#include <mutex>

#define MAX_MESSAGE_LEN 1048

namespace Modelec
{
    class SerialListener
    {
    protected:
        bool status_;
        int bauds_;
        std::string serial_port_;
        int max_message_len_;

        boost::asio::io_service io_;
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

        SerialListener();

        SerialListener(const std::string& name, int bauds, const std::string& serial_port,
                       int max_message_len);

        virtual ~SerialListener();

        void close();
        void open(const std::string& name, int bauds, const std::string& serial_port,
                       int max_message_len);

        void SetMaxMessageLen(int max_message_len) { max_message_len_ = max_message_len; }
        bool IsOk() const { return status_; }

        void SetOk() { status_ = true; }

        virtual void write(const std::string& msg);

        virtual void read(const std::string& msg);
    };
} // namespace Modelec