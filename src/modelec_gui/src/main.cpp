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
