#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace Modelec
{
class PCBAlimInterface : public rclcpp::Node
{
public:
    PCBAlimInterface();

private:

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pcb_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pcb_subscriber_;

    void PCBCallback(const std_msgs::msg::String::SharedPtr msg);

    void SendToPCB(const std::string &data);

    void GetData(const std::string &elem, const std::string& value);

    void SendOrder(const std::string &elem, const std::string& data, const std::string& value);

    // get data
    void GetEmergencyStopButtonState();
    void GetEntryVoltage(int entry);
    void GetEntryCurrent(int entry);
    void GetEntryState(int entry);
    void GetEntryIsValide(int entry);
    void GetPCBTemperature();
    
    void GetOutput5VState();
    void GetOutput5VVoltage();
    void GetOutput5VCurrent();

    void GetOutput5V1State();
    void GetOutput5V1Voltage();
    void GetOutput5V1Current();

    void GetOutput12VState();
    void GetOutput12VVoltage();
    void GetOutput12VCurrent();

    void GetOutput24VState();
    void GetOutput24VVoltage();
    void GetOutput24VCurrent();

    void SetSoftwareEmergencyStop(bool state);
    void Set5VState(bool state);
    void Set12VState(bool state);
    void Set24VState(bool state);
};
}  // namespace Modelec
