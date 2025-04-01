#include <modelec/pcb_alim_interface.hpp>
#include <modelec_interface/srv/add_serial_listener.hpp>

namespace Modelec
{
    PCBAlimInterface::PCBAlimInterface() : Node("pcb_alim_interface")
    {
        // Service to create a new serial listener
        auto request = std::make_shared<modelec_interface::srv::AddSerialListener::Request>();
        request->name = "pcb_alim";
        request->bauds = 115200;
        request->serial_port = "/dev/serial0"; // TODO : check the real serial port
        auto client = this->create_client<modelec_interface::srv::AddSerialListener>("add_serial_listener");
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
        auto result = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
            if (!result.get()->success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to add serial listener");
            } else {
            RCLCPP_INFO(this->get_logger(), "Added serial listener");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }

        pcb_publisher_ = this->create_publisher<std_msgs::msg::String>(result.get()->subscriber, 10);
        pcb_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            result.get()->publisher, 10,
        std::bind(&PCBAlimInterface::PCBCallback, this, std::placeholders::_1));
    }

    void PCBAlimInterface::PCBCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());
    }

    void PCBAlimInterface::SendToPCB(const std::string &data)
    {
        auto message = std_msgs::msg::String();
        message.data = data;
        pcb_publisher_->publish(message);
    }

    void PCBAlimInterface::GetData(const std::string &elem, const std::string& value)
    {
        SendToPCB("GET;" + elem + ";" + value);
    }

    void PCBAlimInterface::SendOrder(const std::string &elem, const std::string& data, const std::string& value)
    {
        SendToPCB("SET;" + elem + ";" + data + ";" + value);
    }

    void PCBAlimInterface::GetEmergencyStopButtonState()
    {
        GetData("BAU", "STATE");
    }

    void PCBAlimInterface::GetEntryVoltage(int entry) {
        GetData("IN" +  std::to_string(entry), "VOLT");
    }

    void PCBAlimInterface::GetEntryCurrent(int entry) {
        GetData("IN" +  std::to_string(entry), "AMP");
    }

    void PCBAlimInterface::GetEntryState(int entry) {
        GetData("IN" +  std::to_string(entry), "STATE");
    }

    void PCBAlimInterface::GetEntryIsValide(int entry) {
        GetData("IN" +  std::to_string(entry), "VALID");
    }

    void PCBAlimInterface::GetPCBTemperature() {
        GetData("TEMP", "CELS");
    }

    
    void PCBAlimInterface::GetOutput5VState() {
        GetData("OUT5V", "STATE");
    }
    
    void PCBAlimInterface::GetOutput5VVoltage() {
        GetData("OUT5V", "VOLT");
    }
    
    void PCBAlimInterface::GetOutput5VCurrent() {
        GetData("OUT5V", "AMP");
    }


    void PCBAlimInterface::GetOutput5V1State() {
        GetData("OUT5V1", "STATE");
    }
    
    void PCBAlimInterface::GetOutput5V1Voltage() {
        GetData("OUT5V1", "VOLT");
    }
    
    void PCBAlimInterface::GetOutput5V1Current() {
        GetData("OUT5V1", "AMP");
    }


    
    void PCBAlimInterface::GetOutput12VState() {
        GetData("OUT12V", "STATE");
    }
    
    void PCBAlimInterface::GetOutput12VVoltage() {
        GetData("OUT12V", "VOLT");
    }
    
    void PCBAlimInterface::GetOutput12VCurrent() {
        GetData("OUT12V", "AMP");
    }


    void PCBAlimInterface::GetOutput24VState() {
        GetData("OUT24V", "STATE");
    }
    
    void PCBAlimInterface::GetOutput24VVoltage() {
        GetData("OUT24V", "VOLT");
    }
    
    void PCBAlimInterface::GetOutput24VCurrent() {
        GetData("OUT24V", "AMP");
    }


    void PCBAlimInterface::SetSoftwareEmergencyStop(bool state) {
        SendOrder("EMG", "STATE", state ? "1" : "0");
    }
    
    void PCBAlimInterface::Set5VState(bool state) {
        SendOrder("OUT5V", "STATE", state ? "1" : "0");
    }
    
    void PCBAlimInterface::Set12VState(bool state) {
        SendOrder("OUT12V", "STATE", state ? "1" : "0");
    }
    
    void PCBAlimInterface::Set24VState(bool state) {
        SendOrder("OUT24V", "STATE", state ? "1" : "0");
    }
} // namespace Modelec

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Modelec::PCBAlimInterface>());
    rclcpp::shutdown();
    return 0;
}
