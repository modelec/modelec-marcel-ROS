#pragma once

#include <rclcpp/rclcpp.hpp>

#include <queue>
#include <mutex>
#include <future>

#include <std_msgs/msg/string.hpp>
#include <modelec_interface/msg/odometry_pos.hpp>
#include <modelec_interface/msg/odometry_speed.hpp>
#include <modelec_interface/msg/odometry_to_f.hpp>
#include <modelec_interface/msg/odometry_waypoint_reach.hpp>
#include <modelec_interface/msg/odometry_add_waypoint.hpp>
#include <modelec_interface/msg/odometry_start.hpp>

#include <modelec_interface/srv/odometry_position.hpp>
#include <modelec_interface/srv/odometry_speed.hpp>
#include <modelec_interface/srv/odometry_to_f.hpp>
#include <modelec_interface/srv/add_serial_listener.hpp>
#include <modelec_interface/srv/odometry_start.hpp>

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

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pcb_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pcb_subscriber_;

    void PCBCallback(const std_msgs::msg::String::SharedPtr msg);

    rclcpp::Publisher<modelec_interface::msg::OdometryPos>::SharedPtr odo_pos_publisher_;
    rclcpp::Publisher<modelec_interface::msg::OdometrySpeed>::SharedPtr odo_speed_publisher_;
    rclcpp::Publisher<modelec_interface::msg::OdometryToF>::SharedPtr odo_tof_publisher_;
    rclcpp::Publisher<modelec_interface::msg::OdometryWaypointReach>::SharedPtr odo_waypoint_reach_publisher_;

    rclcpp::Subscription<modelec_interface::msg::OdometryAddWaypoint>::SharedPtr odo_add_waypoint_subscriber_;
    rclcpp::Subscription<modelec_interface::msg::OdometryPos>::SharedPtr odo_set_pos_subscriber_;

    void AddWaypointCallback(const modelec_interface::msg::OdometryAddWaypoint::SharedPtr msg) const;
    void SetPosCallback(const modelec_interface::msg::OdometryPos::SharedPtr msg) const;

    rclcpp::Service<modelec_interface::srv::OdometryToF>::SharedPtr get_tof_service_;
    rclcpp::Service<modelec_interface::srv::OdometrySpeed>::SharedPtr get_speed_service_;
    rclcpp::Service<modelec_interface::srv::OdometryPosition>::SharedPtr get_position_service_;
    rclcpp::Service<modelec_interface::srv::OdometryStart>::SharedPtr set_start_service_;

    // Promises and mutexes to synchronize service responses asynchronously
    std::queue<std::promise<long>> tof_promises_;
    std::mutex tof_mutex_;

    std::queue<std::promise<OdometryData>> speed_promises_;
    std::mutex speed_mutex_;

    std::queue<std::promise<OdometryData>> pos_promises_;
    std::mutex pos_mutex_;

    std::queue<std::promise<bool>> start_promises_;
    std::mutex start_mutex_;

    // Service handlers using async wait with promises
    void HandleGetTof(const std::shared_ptr<modelec_interface::srv::OdometryToF::Request> request,
                      std::shared_ptr<modelec_interface::srv::OdometryToF::Response> response);

    void HandleGetSpeed(const std::shared_ptr<modelec_interface::srv::OdometrySpeed::Request> request,
                        std::shared_ptr<modelec_interface::srv::OdometrySpeed::Response> response);

    void HandleGetPosition(const std::shared_ptr<modelec_interface::srv::OdometryPosition::Request> request,
                           std::shared_ptr<modelec_interface::srv::OdometryPosition::Response> response);

    void HandleGetStart(const std::shared_ptr<modelec_interface::srv::OdometryStart::Request> request,
                           std::shared_ptr<modelec_interface::srv::OdometryStart::Response> response);

    // Resolving methods called by subscriber callback
    void ResolveToFRequest(long distance);
    void ResolveSpeedRequest(const OdometryData& speed);
    void ResolvePositionRequest(const OdometryData& position);
    void ResolveStartRequest(bool start);

public:
    void SendToPCB(const std::string &data) const;
    void SendToPCB(const std::string& order, const std::string& elem, const std::vector<std::string>& data = {}) const;

    void GetData(const std::string& elem, const std::vector<std::string>& data = {}) const;
    void SendOrder(const std::string &elem, const std::vector<std::string> &data = {}) const;

    void GetPos() const;
    void GetSpeed() const;
    void GetToF(const int &tof) const;

    void SetRobotPos(const modelec_interface::msg::OdometryPos::SharedPtr msg) const;
    void SetRobotPos(long x, long y, long theta) const;

    void AddWaypoint(modelec_interface::msg::OdometryAddWaypoint::SharedPtr msg) const;
    void AddWaypoint(int index, bool IsStopPoint, long x, long y, long theta) const;

    void SetStart(const modelec_interface::msg::OdometryStart::SharedPtr msg) const;
    void SetStart(bool start) const;
};

} // namespace Modelec
