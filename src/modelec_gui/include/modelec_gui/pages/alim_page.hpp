#pragma once

#include <QWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QSpinBox>

#include <rclcpp/rclcpp.hpp>

namespace ModelecGUI
{
    class AlimPage : public QWidget
    {
        Q_OBJECT

    public:
        AlimPage(rclcpp::Node::SharedPtr node, QWidget* parent = nullptr);
        ~AlimPage() override;

        rclcpp::Node::SharedPtr get_node() const { return node_; }

    private:

        rclcpp::Node::SharedPtr node_;
    };
} // namespace Modelec
