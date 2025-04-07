#pragma once

#include <QApplication>
#include <QWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <modelec_interface/msg/odometry_pos.hpp>
#include <modelec_interface/srv/odometry_speed.hpp>

class ROS2QtGUI : public QWidget {
  Q_OBJECT

public:
  explicit ROS2QtGUI(QWidget *parent = nullptr);
  ~ROS2QtGUI() override; // Explicitly declare destructor

  rclcpp::Node::SharedPtr get_node() const { return node_; }

private:
  QLineEdit *xBox_, *yBox_, *thetaBox_;
  QVBoxLayout *mainLayout_;
  QHBoxLayout *posLayout_;
  QPushButton *askSpeed_;
  QLineEdit *xSpeedBox_, *ySpeedBox_, *thetaSpeedBox_;
  QHBoxLayout *speedLayout_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<modelec_interface::msg::OdometryPos>::SharedPtr sub_;

  // client
  rclcpp::Client<modelec_interface::srv::OdometrySpeed>::SharedPtr client_;

  void PositionCallback(const modelec_interface::msg::OdometryPos::SharedPtr msg);
};
