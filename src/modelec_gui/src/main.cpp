#include <rclcpp/rclcpp.hpp>
#include <QApplication>
#include <QThread>
#include "modelec_gui/modelec_gui.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <modelec_utils/config.hpp>

int main(int argc, char **argv)
{
    // Initialize the Qt application
    QApplication app(argc, argv);

    // Initialize ROS 2
    rclcpp::init(argc, argv);

    std::string config_path = ament_index_cpp::get_package_share_directory("modelec_strat") + "/data/config.xml";
    if (!Modelec::Config::load(config_path))
    {
        RCLCPP_ERROR(rclcpp::get_logger("modelec_gui"), "Failed to load configuration file: %s", config_path.c_str());
    }

    // Create the node only once
    auto node = rclcpp::Node::make_shared("qt_gui_node");

    // Create the GUI and pass the node to it
    ModelecGUI::ROS2QtGUI window(node);  // Pass the node to the GUI component
    window.show();

    // Create an executor for ROS 2 to manage the node
    rclcpp::executors::MultiThreadedExecutor executor;

    // Add the node to the executor once
    executor.add_node(node);

    // Run ROS 2 in a separate thread
    std::thread ros_thread([&executor]() {
        executor.spin();  // Execute the node's callbacks
    });

    // Start the Qt application event loop
    int ret = app.exec();

    // Ensure the ROS 2 executor thread ends correctly
    ros_thread.join();
    rclcpp::shutdown(); // Shutdown ROS 2

    return ret; // Return the application result
}
