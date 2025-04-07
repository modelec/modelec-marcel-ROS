#include "modelec_gui/modelec_gui.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  // Create the ROS2QtGUI instance, which is the Qt GUI and ROS node
  auto gui = std::make_shared<ROS2QtGUI>();
  gui->show();

  // Create and start the ROS 2 executor in a separate thread
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  exec->add_node(gui->get_node());  // Ensure this is the correct way to add your node

  std::thread spin_thread([exec]() { exec->spin(); });

  int result = app.exec();

  // Shutdown ROS 2 and join the thread
  exec->cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return result;
}