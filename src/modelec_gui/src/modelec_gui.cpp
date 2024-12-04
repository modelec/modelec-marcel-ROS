#include "modelec_gui/modelec_gui.hpp"

ROS2QtGUI::ROS2QtGUI(std::shared_ptr<rclcpp::Node> node, QWidget *parent)
    : QWidget(parent), node_(node) {
  publisher_ = node_->create_publisher<std_msgs::msg::String>("ui_topic", 10);

  QVBoxLayout *layout = new QVBoxLayout(this);
  textBox_ = new QLineEdit(this);
  sendButton_ = new QPushButton("Send Message", this);

  layout->addWidget(textBox_);
  layout->addWidget(sendButton_);
  setLayout(layout);

  connect(sendButton_, &QPushButton::clicked, this, &ROS2QtGUI::onSendClicked);
}

ROS2QtGUI::~ROS2QtGUI() = default;

void ROS2QtGUI::onSendClicked() {
  std_msgs::msg::String message;
  message.data = textBox_->text().toStdString();
  publisher_->publish(message);
  RCLCPP_INFO(node_->get_logger(), "Published: '%s'", message.data.c_str());
}