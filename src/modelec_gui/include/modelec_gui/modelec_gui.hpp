#pragma once

#include <QApplication>
#include <QWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class ROS2QtGUI : public QWidget {
  Q_OBJECT

public:
  explicit ROS2QtGUI(std::shared_ptr<rclcpp::Node> node, QWidget *parent = nullptr);
  ~ROS2QtGUI() override; // Explicitly declare destructor

private slots:
  void onSendClicked();

private:
  QLineEdit *textBox_;
  QPushButton *sendButton_;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};
