#pragma once

#include <modelec_com/serial_listener.hpp>

#include <rclcpp/rclcpp.hpp>

#include <future>

#include <modelec_interfaces/msg/odometry_pos.hpp>
#include <modelec_interfaces/msg/odometry_speed.hpp>
#include <modelec_interfaces/msg/odometry_to_f.hpp>
#include <modelec_interfaces/msg/odometry_waypoint.hpp>
#include <modelec_interfaces/msg/odometry_waypoints.hpp>
#include <modelec_interfaces/msg/odometry_start.hpp>
#include <modelec_interfaces/msg/odometry_pid.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>

#include <sensor_msgs/msg/joy.hpp>

#include "modelec_utils/config.hpp"

namespace Modelec
{
    class PCBOdoInterface : public rclcpp::Node, public SerialListener
    {
    public:
        PCBOdoInterface();

        ~PCBOdoInterface() override;

        struct OdometryData
        {
            long x;
            long y;
            double theta;
        };

        struct PIDData
        {
            float p = 0.0f;
            float i = 0.0f;
            float d = 0.0f;
            std::optional<float> min;
            std::optional<float> max;
        };

    private:
        void read(const std::string& msg) override;

        rclcpp::Publisher<modelec_interfaces::msg::OdometryPos>::SharedPtr odo_pos_publisher_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr odo_get_pos_sub_;
        rclcpp::Publisher<modelec_interfaces::msg::OdometrySpeed>::SharedPtr odo_speed_publisher_;
        rclcpp::Publisher<modelec_interfaces::msg::OdometryToF>::SharedPtr odo_tof_publisher_;
        rclcpp::Publisher<modelec_interfaces::msg::OdometryWaypoint>::SharedPtr odo_waypoint_reach_publisher_;
        rclcpp::Publisher<modelec_interfaces::msg::OdometryWaypoint>::SharedPtr odo_waypoint_reach_err_publisher_;
        rclcpp::Publisher<modelec_interfaces::msg::OdometryPid>::SharedPtr odo_pid_publisher_;

        rclcpp::Subscription<modelec_interfaces::msg::OdometryWaypoint>::SharedPtr odo_add_waypoint_subscriber_;
        rclcpp::Subscription<modelec_interfaces::msg::OdometryWaypoints>::SharedPtr odo_add_waypoints_subscriber_;
        rclcpp::Subscription<modelec_interfaces::msg::OdometryPos>::SharedPtr odo_set_pos_subscriber_;
        rclcpp::Subscription<modelec_interfaces::msg::OdometryPid>::SharedPtr odo_set_pid_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr odo_ask_active_waypoint_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

        void AddWaypointCallback(const modelec_interfaces::msg::OdometryWaypoint::SharedPtr msg);
        void AddWaypointsCallback(const modelec_interfaces::msg::OdometryWaypoints::SharedPtr msg);
        void SetPosCallback(const modelec_interfaces::msg::OdometryPos::SharedPtr msg);
        void SetPIDCallback(const modelec_interfaces::msg::OdometryPid::SharedPtr msg);

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_odo_sub_;
        bool start_odo_ = false;

        int timeout_ms = 1000;
        int attempt = 5;

        int last_button_11 = 0;

    public:
        void SendToPCB(const std::string& data);
        void SendToPCB(const std::string& order, const std::string& elem,
                       const std::vector<std::string>& data = {});

        void GetData(const std::string& elem, const std::vector<std::string>& data = {});
        void SendOrder(const std::string& elem, const std::vector<std::string>& data = {});

        void GetPos();
        void GetSpeed();
        void GetToF(const int& tof);

        void SetRobotPos(const modelec_interfaces::msg::OdometryPos::SharedPtr msg);
        void SetRobotPos(long x, long y, double theta);

        void AddWaypoints(modelec_interfaces::msg::OdometryWaypoints::SharedPtr msg);
        void AddWaypoint(modelec_interfaces::msg::OdometryWaypoint::SharedPtr msg);
        void AddWaypoint(int index, bool IsStopPoint, long x, long y, double theta);

        void SetStart(const modelec_interfaces::msg::OdometryStart::SharedPtr msg);
        void SetStart(bool start);

        void GetPID();
        void SetPID(const modelec_interfaces::msg::OdometryPid::SharedPtr msg);
        void SetPID(std::string name, float p, float i, float d, std::optional<float> min = std::nullopt, std::optional<float> max = std::nullopt);
        void SetPID(std::string name, const PIDData& pid_data);

        void SetMotor(int left, int right);
    };

    template<>
    inline PCBOdoInterface::PIDData
    Config::get<PCBOdoInterface::PIDData>(
        const std::string& prefix,
        const PCBOdoInterface::PIDData& default_value, bool)
    {
        PCBOdoInterface::PIDData result;
        result.p = Config::get<float>(prefix + "@p", default_value.p);
        result.i = Config::get<float>(prefix + "@i", default_value.i);
        result.d = Config::get<float>(prefix + "@d", default_value.d);
        if (Config::has(prefix + "@min"))
            result.min = Config::get<float>(prefix + "@min");
        if (Config::has(prefix + "@max"))
            result.max = Config::get<float>(prefix + "@max");
        return result;
    }

} // namespace Modelec
