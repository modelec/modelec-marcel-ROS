#pragma once

#include <QWidget>
#include <QLabel>
#include <QPainter>
#include <QPushButton>
#include <QSvgRenderer>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include <rclcpp/rclcpp.hpp>

#include <modelec_interfaces/msg/odometry_add_waypoint.hpp>
#include <modelec_interfaces/msg/odometry_pos.hpp>

namespace ModelecGUI {
    class MapPage : public QWidget
    {
        Q_OBJECT
    public:
        MapPage(rclcpp::Node::SharedPtr node, QWidget *parent = nullptr);

        rclcpp::Node::SharedPtr get_node() const { return node_; }

        void setPlaymatGrid();

        void setPlaymatMap();

    protected:
        void paintEvent(QPaintEvent*) override;

        void mousePressEvent(QMouseEvent* event) override;

        void onStartButtonClicked();

        void onOdometryReceived(const modelec_interfaces::msg::OdometryPos::SharedPtr msg);

        QSvgRenderer* renderer;

        QPushButton* startButton;

        QVBoxLayout* v_layout;
        QHBoxLayout* h_layout;

        QPoint robotPosPoint = QPoint(0, 0);
        std::vector<QPoint> qpoints;
        std::vector<modelec_interfaces::msg::OdometryAddWaypoint> points;

        rclcpp::Node::SharedPtr node_;

        rclcpp::Publisher<modelec_interfaces::msg::OdometryAddWaypoint>::SharedPtr add_waypoint_publisher_;
        rclcpp::Subscription<modelec_interfaces::msg::OdometryPos>::SharedPtr odometry_subscriber_;
    };
}
