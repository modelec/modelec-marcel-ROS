#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <modelec_interface/srv/alim_out.hpp>
#include <modelec_interface/srv/alim_in.hpp>
#include <modelec_interface/srv/alim_bau.hpp>
#include <modelec_interface/srv/alim_emg.hpp>
#include <modelec_interface/srv/alim_temp.hpp>
#include <modelec_interface/msg/alim_emg.hpp>

namespace Modelec
{
class PCBAlimInterface : public rclcpp::Node
{
public:
    PCBAlimInterface();
    rclcpp::CallbackGroup::SharedPtr pcb_callback_group_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> pcb_executor_;
    std::thread pcb_executor_thread_;

    ~PCBAlimInterface() override;

    struct PCBData
    {
        bool success;
        int value;
    };

    struct PCBBau
    {
        bool success;
        bool activate;
    };
private:

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pcb_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pcb_subscriber_;

    void PCBCallback(const std_msgs::msg::String::SharedPtr msg);

    rclcpp::Subscription<modelec_interface::msg::AlimEmg>::SharedPtr pcb_emg_subscriber_;

    void PCBEmgCallback(const modelec_interface::msg::AlimEmg::SharedPtr msg) const;

    rclcpp::Service<modelec_interface::srv::AlimOut>::SharedPtr pcb_out_service_;
    rclcpp::Service<modelec_interface::srv::AlimIn>::SharedPtr pcb_in_service_;
    rclcpp::Service<modelec_interface::srv::AlimBau>::SharedPtr pcb_bau_service_;
    rclcpp::Service<modelec_interface::srv::AlimEmg>::SharedPtr pcb_emg_service_;
    rclcpp::Service<modelec_interface::srv::AlimTemp>::SharedPtr pcb_temp_service_;

    std::queue<std::promise<PCBData>> pcb_out_promises_;
    std::mutex pcb_out_mutex_;

    std::queue<std::promise<PCBData>> pcb_in_promises_;
    std::mutex pcb_in_mutex_;

    std::queue<std::promise<PCBBau>> pcb_bau_promises_;
    std::mutex pcb_bau_mutex_;

    std::queue<std::promise<bool>> pcb_emg_promises_;
    std::mutex pcb_emg_mutex_;

    std::queue<std::promise<PCBData>> pcb_temp_promises_;
    std::mutex pcb_temp_mutex_;

    void HandleGetPCBOutData(
        const std::shared_ptr<modelec_interface::srv::AlimOut::Request> request,
        std::shared_ptr<modelec_interface::srv::AlimOut::Response> response);

    void HandleSetPCBOutData(
        const std::shared_ptr<modelec_interface::srv::AlimOut::Request> request,
        std::shared_ptr<modelec_interface::srv::AlimOut::Response> response);

    void HandleGetPCBInData(
        const std::shared_ptr<modelec_interface::srv::AlimIn::Request> request,
        std::shared_ptr<modelec_interface::srv::AlimIn::Response> response);

    void HandleGetPCBBauData(
        const std::shared_ptr<modelec_interface::srv::AlimBau::Request> request,
        std::shared_ptr<modelec_interface::srv::AlimBau::Response> response);

    void HandleSetPCBEmgData(
        const std::shared_ptr<modelec_interface::srv::AlimEmg::Request> request,
        std::shared_ptr<modelec_interface::srv::AlimEmg::Response> response);

    void HandleGetPCBTempData(
        const std::shared_ptr<modelec_interface::srv::AlimTemp::Request> request,
        std::shared_ptr<modelec_interface::srv::AlimTemp::Response> response);

    void ResolveGetPCBOutRequest(const PCBData& value);
    void ResolveSetPCBOutRequest(const PCBData& value);
    void ResolveGetPCBInRequest(const PCBData& value);
    void ResolveGetPCBBauRequest(const PCBBau& value);
    void ResolveSetPCBEmgRequest(bool value);
    void ResolveGetPCBTempRequest(const PCBData& value);

public:
    void SendToPCB(const std::string &data) const;
    void SendToPCB(const std::string& order, const std::string& elem, const std::vector<std::string>& data = {}) const;

    void GetData(const std::string& elem, const std::vector<std::string>& data = {}) const;
    void SendOrder(const std::string &elem, const std::vector<std::string> &data = {}) const;

};
}  // namespace Modelec
