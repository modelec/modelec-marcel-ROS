#include <modelec_sensors/lidar_controller.hpp>

namespace Modelec {
    LidarController::LidarController() : Node("lidar_controller") {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                processLidarData(msg);
            });
    }

    void LidarController::processLidarData(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Process the received LiDAR data
        RCLCPP_INFO(this->get_logger(), "Received LiDAR data: %s", msg->header.frame_id.c_str());
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Modelec::LidarController>());
    rclcpp::shutdown();
    return 0;
}