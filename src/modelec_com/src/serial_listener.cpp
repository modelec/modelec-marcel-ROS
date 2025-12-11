#include <modelec_com/serial_listener.hpp>
#include <modelec_utils/utils.hpp>

namespace Modelec
{
    SerialListener::SerialListener() : io_(), port_(io_)
    {
    }

    SerialListener::SerialListener(const std::string& name, int bauds, const std::string& serial_port,
                                   int max_message_len) : io_(), port_(io_)
    {
        open(name, bauds, serial_port, max_message_len);
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

    void SerialListener::open(const std::string& name, int bauds, const std::string& serial_port,
               int max_message_len) {
       this->name_ = name;
       this->bauds_ = bauds;
       this->serial_port_ = serial_port;
       this->max_message_len_ = max_message_len;

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
                            // RCLCPP_INFO(rclcpp::get_logger("SerialListener"), "Received message: %s", mess.c_str());
                            /*auto msg = std_msgs::msg::String();
                            msg.data = mess;
                            if (publisher_)
                            {
                                publisher_->publish(msg);
                            }*/
                            read(mess);
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

    void SerialListener::write(const std::string& msg)
    {
        std::lock_guard<std::mutex> lock(write_mutex_);
        bool write_in_progress = !write_queue_.empty();
        write_queue_.push_back(msg);

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

    void SerialListener::read(const std::string&) {
        // Default implementation does nothing
    }
} // namespace Modelec