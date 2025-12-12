#pragma once

#include <modelec_com/serial_listener.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <modelec_interfaces/msg/action_asc_pos.hpp>
#include <modelec_interfaces/msg/action_relay_state_array.hpp>
#include <modelec_interfaces/msg/action_servo_pos_array.hpp>
#include <modelec_interfaces/msg/action_servo_timed_array.hpp>
#include <modelec_interfaces/msg/action_servo_timed.hpp>

#define TIMER_SERVO_TIMED_MS 10

namespace Modelec
{
    struct ServoTimedSet
    {
        modelec_interfaces::msg::ActionServoTimed servo_timed;
        rclcpp::Time start_time;
        rclcpp::Time end_time;

        bool active = false;
    };

    class PCBActionInterface : public rclcpp::Node, public SerialListener
    {
    public:

        PCBActionInterface();

        ~PCBActionInterface() override;

    protected:
        std::map<int, int> asc_value_mapper_;

        int asc_state_ = 0;
        std::map<int, double> servo_value_;
        std::map<int, bool> relay_value_;

        std::map<int, ServoTimedSet> servo_timed_buffer_;

        rclcpp::TimerBase::SharedPtr servo_timed_timer_;

    private:
        void read(const std::string& msg) override;

        rclcpp::Subscription<modelec_interfaces::msg::ActionAscPos>::SharedPtr asc_get_sub_;
        rclcpp::Publisher<modelec_interfaces::msg::ActionAscPos>::SharedPtr asc_get_res_pub_;
        rclcpp::Subscription<modelec_interfaces::msg::ActionAscPos>::SharedPtr asc_set_sub_;
        rclcpp::Publisher<modelec_interfaces::msg::ActionAscPos>::SharedPtr asc_set_res_pub_;
        rclcpp::Subscription<modelec_interfaces::msg::ActionAscPos>::SharedPtr asc_move_sub_;
        rclcpp::Publisher<modelec_interfaces::msg::ActionAscPos>::SharedPtr asc_move_res_pub_;

        rclcpp::Subscription<modelec_interfaces::msg::ActionServoPosArray>::SharedPtr servo_get_sub_;
        rclcpp::Publisher<modelec_interfaces::msg::ActionServoPosArray>::SharedPtr servo_get_res_pub_;
        rclcpp::Subscription<modelec_interfaces::msg::ActionServoPosArray>::SharedPtr servo_move_sub_;
        rclcpp::Publisher<modelec_interfaces::msg::ActionServoPosArray>::SharedPtr servo_move_res_pub_;

        rclcpp::Subscription<modelec_interfaces::msg::ActionServoTimedArray>::SharedPtr servo_move_timed_sub_;
        rclcpp::Publisher<modelec_interfaces::msg::ActionServoTimedArray>::SharedPtr servo_move_timed_res_pub_;

        rclcpp::Subscription<modelec_interfaces::msg::ActionRelayStateArray>::SharedPtr relay_get_sub_;
        rclcpp::Publisher<modelec_interfaces::msg::ActionRelayStateArray>::SharedPtr relay_get_res_pub_;
        rclcpp::Subscription<modelec_interfaces::msg::ActionRelayStateArray>::SharedPtr relay_move_sub_;
        rclcpp::Publisher<modelec_interfaces::msg::ActionRelayStateArray>::SharedPtr relay_move_res_pub_;


        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr tir_start_pub_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr tir_arm_pub_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr tir_disarm_pub_;

        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr tir_start_sub_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr tir_arm_sub_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr tir_disarm_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tir_arm_set_sub_;

    public:
        void SendToPCB(const std::string& data);
        void SendToPCB(const std::string& order, const std::string& elem,
                       const std::vector<std::string>& data = {});

        // TODO redo thos func to accept arrays, poc without them atm
        void GetData(const std::string& elem, const std::vector<std::string>& data = {});
        void SendOrder(const std::string& elem, const std::vector<std::string>& data = {});
        void SendMove(const std::string& elem, const std::vector<std::string>& data = {});
        void RespondEvent(const std::string& elem, const std::vector<std::string>& data = {});
    };
}