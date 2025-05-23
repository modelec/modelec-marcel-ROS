#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <modelec_interfaces/srv/add_serial_listener.hpp>
#include <modelec_interfaces/msg/action_asc_pos.hpp>
#include <modelec_interfaces/msg/action_relay_state.hpp>
#include <modelec_interfaces/msg/action_servo_pos.hpp>

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
        std::map<int, int> asc_value_mapper_;
        std::map<int, std::map<int, int>> servo_pos_mapper_ = {
            {0, {{0, 0}, {1, 0}, {2, 0}, {3, 0}}},
            {1, {{0, 0}, {1, 0}, {2, 0}, {3, 0}}},
            {2, {{0, 0}, {1, 0}, {2, 0}, {3, 0}}}
        };

        int asc_state_ = 0;
        std::map<int, int> servo_value_;
        std::map<int, bool> relay_value_;

    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pcb_publisher_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pcb_subscriber_;

        void PCBCallback(const std_msgs::msg::String::SharedPtr msg);

        rclcpp::Subscription<modelec_interfaces::msg::ActionAscPos>::SharedPtr asc_get_sub_;
        rclcpp::Subscription<modelec_interfaces::msg::ActionServoPos>::SharedPtr servo_get_sub_;
        rclcpp::Subscription<modelec_interfaces::msg::ActionRelayState>::SharedPtr relay_get_sub_;

        rclcpp::Publisher<modelec_interfaces::msg::ActionAscPos>::SharedPtr asc_get_res_pub_;
        rclcpp::Publisher<modelec_interfaces::msg::ActionServoPos>::SharedPtr servo_get_res_pub_;
        rclcpp::Publisher<modelec_interfaces::msg::ActionRelayState>::SharedPtr relay_get_res_pub_;

        rclcpp::Subscription<modelec_interfaces::msg::ActionAscPos>::SharedPtr asc_set_sub_;
        rclcpp::Subscription<modelec_interfaces::msg::ActionServoPos>::SharedPtr servo_set_sub_;

        rclcpp::Publisher<modelec_interfaces::msg::ActionAscPos>::SharedPtr asc_set_res_pub_;
        rclcpp::Publisher<modelec_interfaces::msg::ActionServoPos>::SharedPtr servo_set_res_pub_;

        rclcpp::Subscription<modelec_interfaces::msg::ActionAscPos>::SharedPtr asc_move_sub_;
        rclcpp::Subscription<modelec_interfaces::msg::ActionServoPos>::SharedPtr servo_move_sub_;
        rclcpp::Subscription<modelec_interfaces::msg::ActionRelayState>::SharedPtr relay_move_sub_;

        rclcpp::Publisher<modelec_interfaces::msg::ActionAscPos>::SharedPtr asc_move_res_pub_;
        rclcpp::Publisher<modelec_interfaces::msg::ActionServoPos>::SharedPtr servo_move_res_pub_;
        rclcpp::Publisher<modelec_interfaces::msg::ActionRelayState>::SharedPtr relay_move_res_pub_;

        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr tir_start_pub_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr tir_arm_pub_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr tir_disarm_pub_;

        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr tir_start_sub_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr tir_arm_sub_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr tir_disarm_sub_;


        bool isOk = false;
    public:
        void SendToPCB(const std::string& data) const;
        void SendToPCB(const std::string& order, const std::string& elem,
                       const std::vector<std::string>& data = {}) const;

        void GetData(const std::string& elem, const std::vector<std::string>& data = {}) const;
        void SendOrder(const std::string& elem, const std::vector<std::string>& data = {}) const;
        void SendMove(const std::string& elem, const std::vector<std::string>& data = {}) const;
        void RespondEvent(const std::string& elem, const std::vector<std::string>& data = {}) const;
    };
}