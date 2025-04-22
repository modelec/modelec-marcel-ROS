#pragma once

#include <QStackedWidget>
#include <QMainWindow>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>


namespace ModelecGUI
{
    class ROS2QtGUI : public QMainWindow {
        Q_OBJECT

      public:
        explicit ROS2QtGUI(rclcpp::Node::SharedPtr node, QWidget *parent = nullptr);
        ~ROS2QtGUI() override; // Explicitly declare destructor

        rclcpp::Node::SharedPtr get_node() const { return node_; }

    protected:

        rclcpp::Node::SharedPtr node_;

        QStackedWidget *stackedWidget;
        void setupMenu();

        QAction* home_action_;
        QAction* test_action_;
        QAction* map_action_;
        QAction* exit_action_;

        QMenu* playmat_map_menu_;
        QAction* playmat_map_;
        QAction* playmat_grid_;

    };
}
