#pragma once

#include <QStackedWidget>
#include <QMainWindow>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <modelec_gui/pages/home_page.hpp>
#include <modelec_gui/pages/test_map_page.hpp>
#include <modelec_gui/pages/odo_page.hpp>
#include <modelec_gui/pages/map_page.hpp>
#include <modelec_gui/pages/action_page.hpp>
#include <modelec_gui/pages/alim_page.hpp>


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

        HomePage* home_page_;
        TestMapPage* test_map_page_;
        OdoPage* odo_page_;
        MapPage* map_page_;
        ActionPage* action_page_;
        AlimPage* alim_page_;

        QAction* home_action_;
        QAction* odo_action_;
        QAction* map_action_;
        QAction* action_action_;
        QAction* alim_action_;
        QAction* exit_action_;

        QMenu* playmat_map_menu_;
        QAction* playmat_map_;
        QAction* playmat_grid_;

        QAction* toggle_show_obstacle_action_;

    };
}
