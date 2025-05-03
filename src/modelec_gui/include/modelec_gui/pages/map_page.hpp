#pragma once

#include <QWidget>
#include <QLabel>
#include <QPainter>
#include <QPushButton>
#include <QSvgRenderer>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/empty.hpp>

#include <modelec_interfaces/msg/odometry_add_waypoint.hpp>
#include <modelec_interfaces/msg/odometry_pos.hpp>
#include <modelec_interfaces/srv/map.hpp>
#include <modelec_interfaces/srv/map_size.hpp>
#include <modelec_interfaces/msg/obstacle.hpp>
#include <modelec_interfaces/msg/odometry_waypoint_reach.hpp>

#include <std_msgs/msg/bool.hpp>

namespace ModelecGUI {
    class MapPage : public QWidget
    {
        Q_OBJECT

    public:
        MapPage(rclcpp::Node::SharedPtr node, QWidget *parent = nullptr);

        rclcpp::Node::SharedPtr get_node() const { return node_; }

        void setPlaymatGrid();

        void setPlaymatMap();

        void toggleShowObstacle();

    protected:
        void paintEvent(QPaintEvent*) override;

        void mousePressEvent(QMouseEvent* event) override;

        void mouseReleaseEvent(QMouseEvent* event) override;

        void mouseDoubleClickEvent(QMouseEvent* event) override;

        void mouseMoveEvent(QMouseEvent* event) override;

        void onOdometryReceived(const modelec_interfaces::msg::OdometryPos::SharedPtr msg);

        void OnObstacleReceived(const modelec_interfaces::msg::Obstacle::SharedPtr msg);

        void resizeEvent(QResizeEvent* event) override;

        QSvgRenderer* renderer;

        QVBoxLayout* v_layout;
        QHBoxLayout* h_layout;

        QPushButton* tirette_button_;

        modelec_interfaces::msg::OdometryPos robotPos;
        std::vector<QPoint> qpoints;
        //std::vector<modelec_interfaces::msg::OdometryAddWaypoint> points;
        modelec_interfaces::msg::OdometryPos go_to_point;

        bool lastWapointWasEnd = true;

        std::map<int, modelec_interfaces::msg::Obstacle> obstacle_;
        bool show_obstacle_ = true;

        int map_width_ = 0;
        int map_height_ = 0;

        int robot_length_ = 0;
        int robot_width_ = 0;
        int enemy_length_ = 0;
        int enemy_width_ = 0;

        float ratioBetweenMapAndWidgetX_;
        float ratioBetweenMapAndWidgetY_;

        rclcpp::Node::SharedPtr node_;

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tirette_pub_;

        rclcpp::Subscription<modelec_interfaces::msg::OdometryAddWaypoint>::SharedPtr add_waypoint_sub_;

        rclcpp::Subscription<modelec_interfaces::msg::OdometryPos>::SharedPtr odometry_sub_;
        rclcpp::Publisher<modelec_interfaces::msg::OdometryPos>::SharedPtr go_to_pub_;
        rclcpp::Client<modelec_interfaces::srv::MapSize>::SharedPtr map_client_;
        rclcpp::Subscription<modelec_interfaces::msg::Obstacle>::SharedPtr obstacle_added_sub_;
        rclcpp::Subscription<modelec_interfaces::msg::Obstacle>::SharedPtr obstacle_removed_sub_;
        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr ask_map_obstacle_client_;

        modelec_interfaces::msg::OdometryPos enemy_pos_;
        rclcpp::Publisher<modelec_interfaces::msg::OdometryPos>::SharedPtr enemy_pos_pub_;
    };
}
