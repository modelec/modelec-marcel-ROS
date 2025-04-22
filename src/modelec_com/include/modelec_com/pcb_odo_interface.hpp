#pragma once

#include <rclcpp/rclcpp.hpp>

#include <queue>
#include <mutex>
#include <future>

#include <std_msgs/msg/string.hpp>

#include <modelec_interfaces/msg/odometry_pos.hpp>
#include <modelec_interfaces/msg/odometry_speed.hpp>
#include <modelec_interfaces/msg/odometry_to_f.hpp>
#include <modelec_interfaces/msg/odometry_waypoint_reach.hpp>
#include <modelec_interfaces/msg/odometry_add_waypoint.hpp>
#include <modelec_interfaces/msg/odometry_start.hpp>
#include <modelec_interfaces/msg/odometry_pid.hpp>

#include <modelec_interfaces/srv/odometry_position.hpp>
#include <modelec_interfaces/srv/odometry_speed.hpp>
#include <modelec_interfaces/srv/odometry_to_f.hpp>
#include <modelec_interfaces/srv/add_serial_listener.hpp>
#include <modelec_interfaces/srv/odometry_start.hpp>
#include <modelec_interfaces/srv/odometry_get_pid.hpp>
#include <modelec_interfaces/srv/odometry_set_pid.hpp>
#include <modelec_interfaces/srv/odometry_add_waypoint.hpp>

namespace Modelec {

class PCBOdoInterface : public rclcpp::Node {
public:
    PCBOdoInterface();

    rclcpp::CallbackGroup::SharedPtr pcb_callback_group_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> pcb_executor_;
    std::thread pcb_executor_thread_;
    ~PCBOdoInterface() override;

    struct OdometryData {
        long x;
        long y;
        long theta;
    };

    struct PIDData
    {
        float p;
        float i;
        float d;
    };

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pcb_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pcb_subscriber_;

    void PCBCallback(const std_msgs::msg::String::SharedPtr msg);

    rclcpp::Publisher<modelec_interfaces::msg::OdometryPos>::SharedPtr odo_pos_publisher_;
    rclcpp::Publisher<modelec_interfaces::msg::OdometrySpeed>::SharedPtr odo_speed_publisher_;
    rclcpp::Publisher<modelec_interfaces::msg::OdometryToF>::SharedPtr odo_tof_publisher_;
    rclcpp::Publisher<modelec_interfaces::msg::OdometryWaypointReach>::SharedPtr odo_waypoint_reach_publisher_;
    rclcpp::Publisher<modelec_interfaces::msg::OdometryPid>::SharedPtr odo_pid_publisher_;

    rclcpp::Subscription<modelec_interfaces::msg::OdometryAddWaypoint>::SharedPtr odo_add_waypoint_subscriber_;
    rclcpp::Subscription<modelec_interfaces::msg::OdometryPos>::SharedPtr odo_set_pos_subscriber_;
    rclcpp::Subscription<modelec_interfaces::msg::OdometryPid>::SharedPtr odo_set_pid_subscriber_;

    void AddWaypointCallback(const modelec_interfaces::msg::OdometryAddWaypoint::SharedPtr msg) const;
    void SetPosCallback(const modelec_interfaces::msg::OdometryPos::SharedPtr msg) const;
    void SetPIDCallback(const modelec_interfaces::msg::OdometryPid::SharedPtr msg) const;

    rclcpp::Service<modelec_interfaces::srv::OdometryToF>::SharedPtr get_tof_service_;
    rclcpp::Service<modelec_interfaces::srv::OdometrySpeed>::SharedPtr get_speed_service_;
    rclcpp::Service<modelec_interfaces::srv::OdometryPosition>::SharedPtr get_position_service_;
    rclcpp::Service<modelec_interfaces::srv::OdometryStart>::SharedPtr set_start_service_;
    rclcpp::Service<modelec_interfaces::srv::OdometryGetPid>::SharedPtr get_pid_service_;
    rclcpp::Service<modelec_interfaces::srv::OdometrySetPid>::SharedPtr set_pid_service_;
    rclcpp::Service<modelec_interfaces::srv::OdometryAddWaypoint>::SharedPtr add_waypoint_service_;

    // Promises and mutexes to synchronize service responses asynchronously
    std::queue<std::promise<long>> tof_promises_;
    std::mutex tof_mutex_;

    std::queue<std::promise<OdometryData>> speed_promises_;
    std::mutex speed_mutex_;

    std::queue<std::promise<OdometryData>> pos_promises_;
    std::mutex pos_mutex_;

    std::queue<std::promise<bool>> start_promises_;
    std::mutex start_mutex_;

    std::queue<std::promise<PIDData>> get_pid_promises_;
    std::mutex get_pid_mutex_;

    std::queue<std::promise<bool>> set_pid_promises_;
    std::mutex set_pid_mutex_;

    std::queue<std::promise<bool>> add_waypoint_promises_;
    std::mutex add_waypoint_mutex_;

    // Service handlers using async wait with promises
    void HandleGetTof(const std::shared_ptr<modelec_interfaces::srv::OdometryToF::Request> request,
                      std::shared_ptr<modelec_interfaces::srv::OdometryToF::Response> response);

    void HandleGetSpeed(const std::shared_ptr<modelec_interfaces::srv::OdometrySpeed::Request> request,
                        std::shared_ptr<modelec_interfaces::srv::OdometrySpeed::Response> response);

    void HandleGetPosition(const std::shared_ptr<modelec_interfaces::srv::OdometryPosition::Request> request,
                           std::shared_ptr<modelec_interfaces::srv::OdometryPosition::Response> response);

    void HandleGetStart(const std::shared_ptr<modelec_interfaces::srv::OdometryStart::Request> request,
                           std::shared_ptr<modelec_interfaces::srv::OdometryStart::Response> response);

    void HandleGetPID(const std::shared_ptr<modelec_interfaces::srv::OdometryGetPid::Request> request,
                           std::shared_ptr<modelec_interfaces::srv::OdometryGetPid::Response> response);

    void HandleSetPID(const std::shared_ptr<modelec_interfaces::srv::OdometrySetPid::Request> request,
                           std::shared_ptr<modelec_interfaces::srv::OdometrySetPid::Response> response);

    void HandleAddWaypoint(const std::shared_ptr<modelec_interfaces::srv::OdometryAddWaypoint::Request> request,
                           std::shared_ptr<modelec_interfaces::srv::OdometryAddWaypoint::Response> response);

    // Resolving methods called by subscriber callback
    void ResolveToFRequest(long distance);
    void ResolveSpeedRequest(const OdometryData& speed);
    void ResolvePositionRequest(const OdometryData& position);
    void ResolveStartRequest(bool start);
    void ResolveGetPIDRequest(const PIDData& pid);
    void ResolveSetPIDRequest(bool success);
    void ResolveAddWaypointRequest(bool success);

public:
    void SendToPCB(const std::string &data) const;
    void SendToPCB(const std::string& order, const std::string& elem, const std::vector<std::string>& data = {}) const;

    void GetData(const std::string& elem, const std::vector<std::string>& data = {}) const;
    void SendOrder(const std::string &elem, const std::vector<std::string> &data = {}) const;

    void GetPos() const;
    void GetSpeed() const;
    void GetToF(const int &tof) const;

    void SetRobotPos(const modelec_interfaces::msg::OdometryPos::SharedPtr msg) const;
    void SetRobotPos(long x, long y, long theta) const;

    void AddWaypoint(modelec_interfaces::msg::OdometryAddWaypoint::SharedPtr msg) const;
    void AddWaypoint(int index, bool IsStopPoint, long x, long y, long theta) const;

    void SetStart(const modelec_interfaces::msg::OdometryStart::SharedPtr msg) const;
    void SetStart(bool start) const;

    void GetPID() const;
    void SetPID(const modelec_interfaces::msg::OdometryPid::SharedPtr msg) const;
    void SetPID(float p, float i, float d) const;
};

} // namespace Modelec
