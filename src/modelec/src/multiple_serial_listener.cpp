#include <modelec/multiple_serial_listener.hpp>

namespace Modelec {
    MultipleSerialListener::MultipleSerialListener() : Node("multiple_serial_listener"), io() {
        add_serial_listener_service_ = create_service<modelec_interface::srv::AddSerialListener>("add_serial_listener", std::bind(&MultipleSerialListener::add_serial_listener, this, std::placeholders::_1, std::placeholders::_2));
        timer = create_wall_timer(std::chrono::milliseconds(READ_REFRESH_RATE), [this]() {
            for (auto &listener : serial_listeners) {
                if (listener.second->IsOk())
                    listener.second->read();
            }
        });

        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &parameters) {
                return onParameterChange(parameters);
            });
    }

    MultipleSerialListener::~MultipleSerialListener() {
        for (auto &listener : serial_listeners) {
            listener.second->port_.close();
        }
    }

    void MultipleSerialListener::add_serial_listener(const std::shared_ptr<modelec_interface::srv::AddSerialListener::Request> request, std::shared_ptr<modelec_interface::srv::AddSerialListener::Response> response) {
        if (serial_listeners.find(request->name) != serial_listeners.end()) {
            response->success = true;
            response->publisher = serial_listeners[request->name]->publisher_->get_topic_name();
            response->subscriber = serial_listeners[request->name]->subscriber_->get_topic_name();
            return;
        }

        auto listener = std::make_shared<SerialListener>(request->name, request->bauds, request->serial_port, MAX_MESSAGE_LEN, io);

        if (!listener->IsOk()) {
            response->success = false;
            return;
        }

        listener->publisher_ = create_publisher<std_msgs::msg::String>("raw_data/" + listener->name_, 10);
        listener->subscriber_ = create_subscription<std_msgs::msg::String>( 
            "send_to_serial/" + listener->name_, 10, [listener](std_msgs::msg::String::SharedPtr msg) {
                listener->write(msg->data);
            });

        serial_listeners.insert({request->name, listener});

        response->success = true;
        response->publisher = listener->publisher_->get_topic_name();
        response->subscriber = listener->subscriber_->get_topic_name();
    }

    rcl_interfaces::msg::SetParametersResult MultipleSerialListener::onParameterChange(const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &parameter : parameters) {
            if (parameter.get_name() == "bauds" || parameter.get_name() == "serial_port" || parameter.get_name() == "max_message_len") {
                updateConnection();
            }
        }

        return result;
    }

    void MultipleSerialListener::updateConnection() {
        for (auto &listener : serial_listeners) {
            listener.second->SetMaxMessageLen(get_parameter("max_message_len").as_int());
        }

        read_refresh_rate_ = get_parameter("read_refresh_rate").as_int();
        timer->cancel();
        timer = create_wall_timer(std::chrono::milliseconds(read_refresh_rate_), [this]() {
            for (auto &listener : serial_listeners) {
                listener.second->read();
            }
        });
    }

    MultipleSerialListener::SerialListener::SerialListener(const std::string& name, int bauds, const std::string& serial_port, int max_message_len, boost::asio::io_service& io)
        : bauds_(bauds), serial_port_(serial_port), max_message_len_(max_message_len), io_(io), name_(name), port_(io)
    {
        try {
            port_.open(serial_port_);
            port_.set_option(boost::asio::serial_port_base::baud_rate(bauds_));
            status_ = true;
        } catch (boost::system::system_error &e) {
            RCLCPP_ERROR(rclcpp::get_logger("MultipleSerialListener"), "Failed to open serial port: %s", e.what());
            status_ = false;
            return;
        }
    }

    void MultipleSerialListener::SerialListener::read() {
        if (!status_) return;

        std::vector<char> data(max_message_len_);
        try {
            // Attempt to read data from the serial port
    
            size_t len = port_.read_some(boost::asio::buffer(data.data(), max_message_len_));
            if (len > 0) {
                // Prepare and publish the message
                auto msg = std_msgs::msg::String();
                msg.data = std::string(data.begin(), data.begin() + len);
                publisher_->publish(msg);
            }
        } catch (boost::system::system_error &e) {
            RCLCPP_ERROR(rclcpp::get_logger("SerialListener"), "Read error: %s", e.what());
        }
    }
    

    void MultipleSerialListener::SerialListener::write(const std::string& data) {
        try {
            boost::asio::write(port_, boost::asio::buffer(data));
        } catch (boost::system::system_error &e) {
            RCLCPP_ERROR(rclcpp::get_logger("SerialListener"), "Write error: %s", e.what());
        }
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Modelec::MultipleSerialListener>());
    rclcpp::shutdown();
    return 0;
}
