#include <ament_index_cpp/get_package_share_directory.hpp>
#include <modelec_com/pcb_action_interface.hpp>
#include <modelec_utils/config.hpp>
#include <modelec_utils/utils.hpp>

namespace Modelec
{
    PCBActionInterface::PCBActionInterface() : Node("pcb_action_interface")
    {
        // Service to create a new serial listener
        std::string config_path = ament_index_cpp::get_package_share_directory("modelec_strat") + "/data/config.xml";
        if (!Config::load(config_path))
        {
            RCLCPP_ERROR(get_logger(), "Failed to load config file: %s", config_path.c_str());
        }

        auto request = std::make_shared<modelec_interfaces::srv::AddSerialListener::Request>();
        request->name = Config::get<std::string>("config.usb.pcb.pcb_action.name", "pcb_action");
        request->bauds = Config::get<int>("config.usb.pcb.pcb_action.baudrate", 115200);
        request->serial_port = Config::get<std::string>("config.usb.pcb.pcb_action.port", "/dev/ttyUSB0");

        auto client = this->create_client<modelec_interfaces::srv::AddSerialListener>("add_serial_listener");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
        auto result = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            if (auto res = result.get())
            {
                if (res->success)
                {
                    pcb_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

                    rclcpp::SubscriptionOptions options;
                    options.callback_group = pcb_callback_group_;

                    pcb_subscriber_ = this->create_subscription<std_msgs::msg::String>(
                        res->publisher, 10,
                        [this](const std_msgs::msg::String::SharedPtr msg)
                        {
                            PCBCallback(msg);
                        }, options);

                    pcb_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
                    pcb_executor_->add_callback_group(pcb_callback_group_, this->get_node_base_interface());

                    pcb_executor_thread_ = std::thread([this]()
                    {
                        pcb_executor_->spin();
                    });

                    pcb_publisher_ = this->create_publisher<std_msgs::msg::String>(res->subscriber, 10);
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to add serial listener");
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to ask for a serial listener");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

    PCBActionInterface::~PCBActionInterface()
    {
        pcb_executor_->cancel();
        if (pcb_executor_thread_.joinable())
        {
            pcb_executor_thread_.join();
        }
    }

    void PCBActionInterface::PCBCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_DEBUG(this->get_logger(), "Received message: '%s'", msg->data.c_str());
        std::vector<std::string> tokens = split(msg->data, ';');
    }

    void PCBActionInterface::SendToPCB(const std::string& data) const
    {
        auto message = std_msgs::msg::String();
        message.data = data;
        pcb_publisher_->publish(message);
    }

    void PCBActionInterface::SendToPCB(const std::string& order, const std::string& elem,
        const std::vector<std::string>& data) const
    {
        std::string command = order + ";" + elem;
        for (const auto& d : data)
        {
            command += ";" + d;
        }
        command += "\n";

        SendToPCB(command);
    }

    void PCBActionInterface::GetData(const std::string& elem, const std::vector<std::string>& data) const
    {
        SendToPCB("GET", elem, data);
    }

    void PCBActionInterface::SendOrder(const std::string& elem, const std::vector<std::string>& data) const
    {
        SendToPCB("SET", elem, data);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Modelec::PCBActionInterface>();

    // Increase number of threads explicitly!
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 2 /* or more threads! */);

    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
