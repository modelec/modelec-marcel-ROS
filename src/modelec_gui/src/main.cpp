#include "modelec_gui/modelec_gui.hpp"

int main(int argc, char *argv[]) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ros2_qt_gui_node");

  // Initialize Qt
  QApplication app(argc, argv);
  ROS2QtGUI gui(node);
  gui.setWindowTitle("ROS 2 Qt GUI");
  gui.resize(400, 200);
  gui.show();

  // Run the Qt event loop
  return app.exec();
}