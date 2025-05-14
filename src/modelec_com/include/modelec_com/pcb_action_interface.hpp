#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <modelec_interfaces/srv/add_serial_listener.hpp>

namespace Modelec
{
    class PCBActionInterface : public rclcpp::Node
    {
    public:
        PCBActionInterface();
        rclcpp::CallbackGroup::SharedPtr pcb_callback_group_;
        std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> pcb_executor_;
        std::thread pcb_executor_thread_;

        ~PCBActionInterface() override;

    protected:
        std::map<std::string, int> asc_v_;
        std::map<int, std::map<int, int>> servo_pos_v_;
        std::map<std::string, bool> relay_v_;

    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pcb_publisher_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pcb_subscriber_;

        void PCBCallback(const std_msgs::msg::String::SharedPtr msg);

    public:
        void SendToPCB(const std::string& data) const;
        void SendToPCB(const std::string& order, const std::string& elem,
                       const std::vector<std::string>& data = {}) const;

        void GetData(const std::string& elem, const std::vector<std::string>& data = {}) const;
        void SendOrder(const std::string& elem, const std::vector<std::string>& data = {}) const;
    };
}