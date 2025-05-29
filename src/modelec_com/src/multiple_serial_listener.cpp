#include "modelec_com/multiple_serial_listener.hpp"
#include <boost/system/error_code.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>
#include <chrono>
#include <thread>
#include <modelec_utils/utils.hpp>

namespace Modelec
{
    SerialListener::SerialListener(const std::string& name, int bauds, const std::string& serial_port,
                                   int max_message_len, boost::asio::io_service& io)
        : bauds_(bauds), serial_port_(serial_port), max_message_len_(max_message_len), io_(io), name_(name), port_(io)
    {
        try
        {
            port_.open(serial_port_);
            port_.set_option(boost::asio::serial_port_base::baud_rate(bauds_));
            status_ = true;
        }
        catch (boost::system::system_error& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("SerialListener"), "Failed to open serial port: %s", e.what());
            status_ = false;
            return;
        }

        read_buffer_.resize(max_message_len_);
        start_async_read();

        io_thread_ = std::thread([this]()
        {
            try
            {
                io_.run();
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("SerialListener"), "IO thread exception: %s", e.what());
            }
        });
    }

    SerialListener::~SerialListener()
    {
        if (status_)
        {
            close();
        }
    }

    void SerialListener::close()
    {
        if (status_)
        {
            if (port_.is_open()) port_.close();
            io_.stop();
            if (io_thread_.joinable()) io_thread_.join();
            status_ = false;
        }
    }

    void SerialListener::start_async_read()
    {
        if (!status_) start_async_read();

        port_.async_read_some(
            boost::asio::buffer(read_buffer_),
            [this](const boost::system::error_code& ec, std::size_t bytes_transferred)
            {
                if (!ec && bytes_transferred > 0)
                {
                    std::string d = std::string(read_buffer_.begin(), read_buffer_.begin() + bytes_transferred);
                    auto allMess = Modelec::split(d, '\n');
                    for (const auto& mess : allMess)
                    {
                        if (!mess.empty())
                        {
                            auto msg = std_msgs::msg::String();
                            msg.data = mess;
                            if (publisher_)
                            {
                                publisher_->publish(msg);
                            }
                        }
                    }

                    start_async_read(); // continue reading
                }
                else
                {
                    RCLCPP_ERROR(rclcpp::get_logger("SerialListener"), "Async read error: %s", ec.message().c_str());
                }
            });
    }

    void SerialListener::write(std_msgs::msg::String::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(write_mutex_);
        bool write_in_progress = !write_queue_.empty();
        write_queue_.push_back(msg->data);

        if (!write_in_progress)
        {
            start_async_write();
        }
    }

    void SerialListener::start_async_write()
    {
        if (write_queue_.empty()) return;

        boost::asio::async_write(
            port_,
            boost::asio::buffer(write_queue_.front()),
            [this](const boost::system::error_code& ec, std::size_t /*length*/)
            {
                std::lock_guard<std::mutex> lock(write_mutex_);
                if (!ec)
                {
                    write_queue_.pop_front();
                    if (!write_queue_.empty())
                    {
                        start_async_write(); // continue writing
                    }
                }
                else
                {
                    RCLCPP_ERROR(rclcpp::get_logger("SerialListener"), "Async write error: %s", ec.message().c_str());
                }
            });
    }

    MultipleSerialListener::MultipleSerialListener()
        : Node("multiple_serial_listener"), io()
    {
        add_serial_listener_service_ = create_service<modelec_interfaces::srv::AddSerialListener>(
            "add_serial_listener", std::bind(&MultipleSerialListener::add_serial_listener, this, std::placeholders::_1,
                                             std::placeholders::_2));
    }

    MultipleSerialListener::~MultipleSerialListener()
    {
        for (auto& listener : serial_listeners)
        {
            listener.second->close();
        }
    }

    void MultipleSerialListener::add_serial_listener(
        const std::shared_ptr<modelec_interfaces::srv::AddSerialListener::Request> request,
        std::shared_ptr<modelec_interfaces::srv::AddSerialListener::Response> response)
    {
        if (serial_listeners.find(request->name) != serial_listeners.end())
        {
            response->success = true;
            response->publisher = serial_listeners[request->name]->publisher_->get_topic_name();
            response->subscriber = serial_listeners[request->name]->subscriber_->get_topic_name();
            return;
        }

        RCLCPP_INFO(rclcpp::get_logger("MultipleSerialListener"), "Adding serial listener: %s", request->name.c_str());

        auto listener = std::make_shared<SerialListener>(request->name, request->bauds, request->serial_port,
                                                         MAX_MESSAGE_LEN, io);

        if (!listener->IsOk())
        {
            response->success = false;
            return;
        }

        listener->publisher_ = create_publisher<std_msgs::msg::String>("raw_data/" + request->name, 10);
        listener->subscriber_ = create_subscription<std_msgs::msg::String>(
            "send_to_serial/" + request->name, rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::BestEffort),
            std::bind(&SerialListener::write, listener.get(), std::placeholders::_1));

        serial_listeners.insert({request->name, listener});

        response->success = true;
        response->publisher = listener->publisher_->get_topic_name();
        response->subscriber = listener->subscriber_->get_topic_name();

        RCLCPP_INFO(rclcpp::get_logger("MultipleSerialListener"), "Serial listener %s fully created",
                    request->name.c_str());
    }

    rcl_interfaces::msg::SetParametersResult MultipleSerialListener::onParameterChange(
        const std::vector<rclcpp::Parameter>& parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& parameter : parameters)
        {
            if (parameter.get_name() == "bauds" || parameter.get_name() == "serial_port" || parameter.get_name() ==
                "max_message_len")
            {
                updateConnection();
            }
        }

        return result;
    }

    void MultipleSerialListener::updateConnection()
    {
        for (auto& listener : serial_listeners)
        {
            listener.second->SetMaxMessageLen(get_parameter("max_message_len").as_int());
        }
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Modelec::MultipleSerialListener>());
    rclcpp::shutdown();
    return 0;
}
