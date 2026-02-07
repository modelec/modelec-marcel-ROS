#include <modelec_gui/pages/test_map_page.hpp>
#include <modelec_utils/utils.hpp>

#include <QMouseEvent>
#include <utility>
#include <cmath>

namespace ModelecGUI
{
    TestMapPage::TestMapPage(rclcpp::Node::SharedPtr node, QWidget* parent) : QWidget(parent),
                                                                              renderer_(new QSvgRenderer(
                                                                                  QString(
                                                                                      ":/img/playmat/2026_FINAL.svg"),
                                                                                  this)),
                                                                              node_(node),
                                                                              robot_texture_(":/img/logo/modelec.png"),
                                                                              top_texture_(":/img/logo/ISEN-Nantes.png"),
                                                                              obs_texture_(":/img/wood.jpg")
    {
        ratioBetweenMapAndWidgetX_ = width() / 3000.0f;
        ratioBetweenMapAndWidgetY_ = height() / 2000.0f;

        qpoints = {};

        map_height_ = node_->get_parameter("map.size.map_height_mm").as_int();
        map_width_ = node_->get_parameter("map.size.map_width_mm").as_int();

        robot_length_ = node_->get_parameter("robot.size.length_mm").as_int();
        robot_width_ = node_->get_parameter("robot.size.width_mm").as_int();

        enemy_length_ = node_->get_parameter("enemy.size.length_mm").as_int();
        enemy_width_ = node_->get_parameter("enemy.size.width_mm").as_int();

        add_waypoint_sub_ = node_->create_subscription<modelec_interfaces::msg::OdometryWaypoint>(
            "odometry/add_waypoint", 100,
            [this](const modelec_interfaces::msg::OdometryWaypoint::SharedPtr msg)
            {
                if (lastWapointWasEnd)
                {
                    qpoints.clear();
                    lastWapointWasEnd = false;

                    qpoints.emplace_back(robotPos.x * ratioBetweenMapAndWidgetX_,
                                             height() - robotPos.y * ratioBetweenMapAndWidgetY_);
                }

                if (msg->is_end)
                {
                    lastWapointWasEnd = true;
                }

                qpoints.emplace_back(msg->x * ratioBetweenMapAndWidgetX_,
                                         height() - msg->y * ratioBetweenMapAndWidgetY_);

                waypoints_dirty_ = true;
                update();
            });

        add_waypoints_sub_ = node_->create_subscription<modelec_interfaces::msg::OdometryWaypoints>(
            "odometry/add_waypoints", 10,
            [this](const modelec_interfaces::msg::OdometryWaypoints::SharedPtr msg)
            {
                qpoints.clear();
                lastWapointWasEnd = false;

                qpoints.emplace_back(robotPos.x * ratioBetweenMapAndWidgetX_,
                                         height() - robotPos.y * ratioBetweenMapAndWidgetY_);

                for (const auto& point : msg->waypoints)
                {
                    qpoints.emplace_back(point.x * ratioBetweenMapAndWidgetX_,
                                             height() - point.y * ratioBetweenMapAndWidgetY_);
                }

                waypoints_dirty_ = true;
                update();
            });

        odometry_sub_ = node_->create_subscription<modelec_interfaces::msg::OdometryPos>("odometry/position", 10,
            [this](const modelec_interfaces::msg::OdometryPos::SharedPtr msg)
            {
                robotPos = *msg;
                update();
            });

        obstacle_added_sub_ = node_->create_subscription<modelec_interfaces::msg::Obstacle>("nav/obstacle/added", 40,
            [this](const modelec_interfaces::msg::Obstacle::SharedPtr msg)
            {
                OnObstacleReceived(msg);
            });

        obstacle_removed_sub_ = node_->create_subscription<modelec_interfaces::msg::Obstacle>(
            "nav/obstacle/removed", 40,
            [this](const modelec_interfaces::msg::Obstacle::SharedPtr msg)
            {
                obstacle_.erase(msg->id);
                obstacles_dirty_ = true;
                update();
            });

        go_to_pub_ = node_->create_publisher<modelec_interfaces::msg::OdometryGoTo>("nav/go_to", 10);

        enemy_pos_pub_ = node_->create_publisher<modelec_interfaces::msg::OdometryPos>("enemy/position", 10);

        enemy_pos_sub_ = node_->create_subscription<modelec_interfaces::msg::OdometryPos>("enemy/position", 10,
            [this](const modelec_interfaces::msg::OdometryPos::SharedPtr msg)
            {
                if (!hasEnemy) hasEnemy = true;

                enemy_pos_ = *msg;
                update();
            });

        // client to nav/map
        map_client_ = node_->create_client<modelec_interfaces::srv::MapSize>("nav/map_size");
        while (!map_client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(node_->get_logger(), "Waiting for the service...");
        }

        auto result = map_client_->async_send_request(std::make_shared<modelec_interfaces::srv::MapSize::Request>());
        if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            if (auto res = result.get())
            {
                map_width_ = res->width;
                map_height_ = res->height;
            }
        }

        ask_map_obstacle_client_ = node_->create_client<std_srvs::srv::Empty>("nav/ask_map_obstacle");
        while (!ask_map_obstacle_client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(node_->get_logger(), "Waiting for the service...");
        }

        auto result2 = ask_map_obstacle_client_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
        rclcpp::spin_until_future_complete(node_->get_node_base_interface(), result2);

        start_lidar_pub_ = node_->create_publisher<std_msgs::msg::Bool>("lidar/start", 10);
    }

    void TestMapPage::setPlaymatGrid()
    {
        renderer_->load(QString(":/img/playmat/grid_v1.svg"));
        bg_dirty_ = true;
        update();
    }

    void TestMapPage::setPlaymatMap()
    {
        renderer_->load(QString(":/img/playmat/2026_FINAL.svg"));
        bg_dirty_ = true;
        update();
    }

    void TestMapPage::toggleShowObstacle()
    {
        show_obstacle_ = !show_obstacle_;
    }

    void TestMapPage::AskMap()
    {
        ask_map_obstacle_client_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
    }

    void TestMapPage::StartLidar() {
        auto msg = std_msgs::msg::Bool();
        msg.data = true;
        start_lidar_pub_->publish(msg);
    }

    void TestMapPage::paintEvent(QPaintEvent* paint_event)
    {
        QWidget::paintEvent(paint_event);

        QPainter painter(this);

        if (bg_dirty_) updateBackgroundCache();
        painter.drawPixmap(0, 0, background_cache_);

        if (show_obstacle_)
        {
            if (obstacles_dirty_) updateObstaclesCache();
            painter.drawPixmap(0, 0, obstacles_cache_);
        }

        if (waypoints_dirty_) updateWaypointsCache();
        painter.drawPixmap(0, 0, waypoints_cache_);

        // --- robot ---
        painter.save();
        painter.translate(robotPos.x * ratioBetweenMapAndWidgetX_,
                          height() - robotPos.y * ratioBetweenMapAndWidgetY_);
        painter.rotate(90 - robotPos.theta * 180.0 / M_PI);

        QRect r(-(robot_width_*ratioBetweenMapAndWidgetX_)/2, -(robot_length_*ratioBetweenMapAndWidgetY_)/2,
                (robot_width_*ratioBetweenMapAndWidgetX_), (robot_length_*ratioBetweenMapAndWidgetY_));

        painter.drawPixmap(r, robot_texture_);
        painter.restore();

        // --- Enemy ---
        if (hasEnemy)
        {
            painter.setBrush(Qt::red);
            painter.drawRect(
                (enemy_pos_.x - enemy_width_/2) * ratioBetweenMapAndWidgetX_,
                height() - (enemy_pos_.y + enemy_length_/2) * ratioBetweenMapAndWidgetY_,
                enemy_width_ * ratioBetweenMapAndWidgetX_,
                enemy_length_ * ratioBetweenMapAndWidgetY_);
        }
    }

    void TestMapPage::mousePressEvent(QMouseEvent* event)
    {
        QWidget::mousePressEvent(event);

        if (event->button() == Qt::LeftButton)
        {
            modelec_interfaces::msg::OdometryGoTo msg;
            msg.x = Modelec::mapValue(event->pos().x(), 0, width(), 0, 3000);
            msg.y = 2000 - Modelec::mapValue(event->pos().y(), 0, height(), 0, 2000);
            msg.theta = atan2(-msg.x, -msg.y);
            msg.close = true;
            if (show_obstacle_)
            {
                msg.mask = modelec_interfaces::msg::OdometryGoTo::FREE | modelec_interfaces::msg::OdometryGoTo::WALL;
            }
            else
            {
                msg.mask = modelec_interfaces::msg::OdometryGoTo::FREE | modelec_interfaces::msg::OdometryGoTo::WALL |
                    modelec_interfaces::msg::OdometryGoTo::OBSTACLE | modelec_interfaces::msg::OdometryGoTo::ENEMY;
            }

            go_to_pub_->publish(msg);
        }
        else if (event->button() == Qt::RightButton)
        {
            modelec_interfaces::msg::OdometryPos msg;
            msg.x = Modelec::mapValue(event->pos().x(), 0, width(), 0, 3000);
            msg.y = 2000 - Modelec::mapValue(event->pos().y(), 0, height(), 0, 2000);
            msg.theta = 0;

            enemy_pos_pub_->publish(msg);
        }
    }

    void TestMapPage::OnObstacleReceived(const modelec_interfaces::msg::Obstacle::SharedPtr msg)
    {
        obstacle_.emplace(msg->id, *msg);
        obstacles_dirty_ = true;
        update();
    }

    void TestMapPage::resizeEvent(QResizeEvent* event)
    {
        QWidget::resizeEvent(event);

        ratioBetweenMapAndWidgetX_ = width() / 3000.0f;
        ratioBetweenMapAndWidgetY_ = height() / 2000.0f;

        bg_dirty_ = true;
        obstacles_dirty_ = true;
        waypoints_dirty_ = true;
        update();
    }

    void TestMapPage::updateBackgroundCache()
    {
        background_cache_ = QPixmap(size());
        background_cache_.fill(Qt::transparent);

        QPainter p(&background_cache_);
        renderer_->render(&p, rect());
        bg_dirty_ = false;
    }

    void TestMapPage::updateObstaclesCache()
    {
        obstacles_cache_ = QPixmap(size());
        obstacles_cache_.fill(Qt::transparent);

        QPainter painter(&obstacles_cache_);
        painter.setRenderHint(QPainter::Antialiasing);

        for (auto& [index, obs] : obstacle_)
        {
            painter.save();

            QPoint pos(obs.x * ratioBetweenMapAndWidgetX_,
                       height() - obs.y * ratioBetweenMapAndWidgetY_);
            painter.translate(pos);
            painter.rotate(90 - obs.theta * 180.0 / M_PI);

            if (obs.type == modelec_interfaces::msg::Obstacle::GRADIN)
            {
                painter.setBrush(QBrush(obs_texture_));
            }
            else if (obs.id == 2)
            {

                auto texture = top_texture_.scaled(obs.width * ratioBetweenMapAndWidgetX_,
                                       obs.height * ratioBetweenMapAndWidgetY_, Qt::KeepAspectRatio);

                QRect imageRect(-(texture.width() / 2), -(texture.height() / 2), texture.width(), texture.height());

                QRect toDraw(-(obs.width * ratioBetweenMapAndWidgetX_ / 2),
                     -(obs.height * ratioBetweenMapAndWidgetY_ / 2),
                     obs.width * ratioBetweenMapAndWidgetX_, obs.height * ratioBetweenMapAndWidgetY_);

                painter.setBrush(Qt::white);
                painter.setPen(Qt::NoPen);
                painter.drawRect(toDraw);

                painter.drawPixmap(imageRect.topLeft(), texture);

                painter.restore();

                continue;
            }
            else if (obs.type == modelec_interfaces::msg::Obstacle::ESTRADE)
            {
                painter.setBrush(Qt::white);
                painter.setPen(Qt::NoPen);
            }
            else
            {
                painter.setBrush(Qt::red);
                painter.setOpacity(0.5);
                painter.setPen(QPen(Qt::red, 5));
            }

            QRect r(-(obs.width * ratioBetweenMapAndWidgetX_ / 2),
                    -(obs.height * ratioBetweenMapAndWidgetY_ / 2),
                     obs.width * ratioBetweenMapAndWidgetX_,
                     obs.height * ratioBetweenMapAndWidgetY_);

            painter.drawRect(r);

            painter.restore();
        }

        obstacles_dirty_ = false;
    }

    void TestMapPage::updateWaypointsCache()
    {
        waypoints_cache_ = QPixmap(size());
        waypoints_cache_.fill(Qt::transparent);

        QPainter painter(&waypoints_cache_);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.setPen(QPen(Qt::red, 2));

        for (size_t i = 0; i + 1 < qpoints.size(); ++i)
            painter.drawLine(qpoints[i], qpoints[i + 1]);

        painter.setPen(Qt::NoPen);
        painter.setBrush(Qt::red);

        for (auto& p : qpoints)
            painter.drawEllipse(p, 5, 5);

        waypoints_dirty_ = false;
    }
}
