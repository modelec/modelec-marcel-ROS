#include <rclcpp/rclcpp.hpp>
#include <QApplication>
#include <QThread>
#include "modelec_gui/modelec_gui.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

int main(int argc, char **argv)
{
    QApplication app(argc, argv);

    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("qt_gui_node");

    node->declare_parameter("map.size.map_width_mm", 3000);
    node->declare_parameter("map.size.map_height_mm", 2000);

    node->declare_parameter("robot.size.length_mm", 200);
    node->declare_parameter("robot.size.width_mm", 300);

    node->declare_parameter("enemy.size.length_mm", 200);
    node->declare_parameter("enemy.size.width_mm", 300);

    ModelecGUI::ROS2QtGUI window(node);
    window.show();

    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(node);

    std::thread ros_thread([&executor]() {
        executor.spin();
    });

    int ret = app.exec();

    executor.cancel();
    rclcpp::shutdown();
    ros_thread.join();

    return ret;
}
