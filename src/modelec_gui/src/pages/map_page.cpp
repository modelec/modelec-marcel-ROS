#include <modelec_gui/pages/map_page.hpp>
#include <modelec_utils/utils.hpp>

#include <QMouseEvent>
#include <utility>
#include <cmath>

#include <modelec_utils/config.hpp>

namespace ModelecGUI
{
    MapPage::MapPage(rclcpp::Node::SharedPtr node, QWidget* parent) : QWidget(parent),
                                                                      renderer_(new QSvgRenderer(
                                                                          QString(":/img/playmat/2026_FINAL.svg"),
                                                                          this)),
                                                                      node_(node),
                                                                      robot_texture_(":/img/logo/modelec.png"),
                                                                      top_texture_(":/img/logo/ISEN-Nantes.png"),
                                                                      obs_texture_(":/img/wood.jpg")
    {
        ratioBetweenMapAndWidgetX_ = width() / 3000.0f;
        ratioBetweenMapAndWidgetY_ = height() / 2000.0f;

        v_layout = new QVBoxLayout(this);

        timer_label_ = new QLabel("00 s", this);
        timer_label_->setAlignment(Qt::AlignCenter);
        timer_label_->setFont(QFont("Arial", 26));
        timer_label_->setStyleSheet("QLabel { color: black; }");
        score_label_ = new QLabel("Score: 0", this);
        score_label_->setAlignment(Qt::AlignCenter);
        score_label_->setFont(QFont("Arial", 26));
        score_label_->setStyleSheet("QLabel { color: black; }");

        h_layout = new QHBoxLayout();
        h_layout->addStretch();
        h_layout->addStretch();
        h_layout->addWidget(score_label_);
        h_layout->addStretch();
        h_layout->addStretch();
        h_layout->addStretch();
        h_layout->addWidget(timer_label_);
        h_layout->addStretch();
        h_layout->addStretch();

        v_layout->addLayout(h_layout);
        v_layout->addStretch();

        this->setLayout(v_layout);

        qpoints = {};

        map_height_ = Modelec::Config::get<int>("config.map.size.map_height_mm", 2000);
        map_width_ = Modelec::Config::get<int>("config.map.size.map_width_mm", 3000);

        robot_length_ = Modelec::Config::get<int>("config.robot.size.length_mm", 300);
        robot_width_ = Modelec::Config::get<int>("config.robot.size.width_mm", 200);

        enemy_length_ = Modelec::Config::get<int>("config.enemy.size.length_mm", 300);
        enemy_width_ = Modelec::Config::get<int>("config.enemy.size.width_mm", 300);

        add_waypoint_sub_ = node_->create_subscription<modelec_interfaces::msg::OdometryWaypoint>(
            "odometry/add_waypoint", 100,
            [this](const modelec_interfaces::msg::OdometryWaypoint::SharedPtr msg)
            {
                if (lastWapointWasEnd)
                {
                    qpoints.clear();
                    lastWapointWasEnd = false;

                    qpoints.push_back(QPoint(robotPos.x * ratioBetweenMapAndWidgetX_,
                                             height() - robotPos.y * ratioBetweenMapAndWidgetY_));
                }

                if (msg->is_end)
                {
                    lastWapointWasEnd = true;
                }

                qpoints.push_back(QPoint(msg->x * ratioBetweenMapAndWidgetX_,
                                         height() - msg->y * ratioBetweenMapAndWidgetY_));

                waypoints_dirty_ = true;
                update();
            });

        add_waypoints_sub_ = node_->create_subscription<modelec_interfaces::msg::OdometryWaypoints>(
        "odometry/add_waypoints", 10,
        [this](const modelec_interfaces::msg::OdometryWaypoints::SharedPtr msg)
        {
            qpoints.clear();
            lastWapointWasEnd = false;

            qpoints.push_back(QPoint(robotPos.x * ratioBetweenMapAndWidgetX_,
                                     height() - robotPos.y * ratioBetweenMapAndWidgetY_));

            for (const auto& point : msg->waypoints)
            {
                qpoints.push_back(QPoint(point.x * ratioBetweenMapAndWidgetX_,
                                         height() - point.y * ratioBetweenMapAndWidgetY_));
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

        score_sub_ = node_->create_subscription<std_msgs::msg::Int64>("strat/score", 10,
                                                                      [this](const std_msgs::msg::Int64::SharedPtr msg)
                                                                      {
                                                                          score_ += msg->data;
                                                                          score_label_->setText(
                                                                              QString("Score: %1").arg(score_));
                                                                      });

        obstacle_added_sub_ = node_->create_subscription<modelec_interfaces::msg::Obstacle>("nav/obstacle/added", 40,
            [this](const modelec_interfaces::msg::Obstacle::SharedPtr msg)
            {
                OnObstacleReceived(msg);
                obstacles_dirty_ = true;
                update();
            });

        obstacle_removed_sub_ = node_->create_subscription<modelec_interfaces::msg::Obstacle>(
            "nav/obstacle/removed", 40,
            [this](const modelec_interfaces::msg::Obstacle::SharedPtr msg)
            {
                obstacle_.erase(msg->id);
                obstacles_dirty_ = true;
                update();
            });

        enemy_pos_sub_ = node_->create_subscription<modelec_interfaces::msg::OdometryPos>("enemy/position", 10,
            [this](const modelec_interfaces::msg::OdometryPos::SharedPtr msg)
            {
                if (!hasEnemy) hasEnemy = true;

                enemy_pos_ = *msg;
                update();
            });

        strat_start_sub_ = node_->create_subscription<std_msgs::msg::Int64>("strat/start_time", 10,
                                                                            [this](
                                                                            const std_msgs::msg::Int64::SharedPtr msg)
                                                                            {
                                                                                isGameStarted_ = true;
                                                                                start_time_ = msg->data;
                                                                            });

        strat_state_sub_ = node_->create_subscription<modelec_interfaces::msg::StratState>("strat/state", 10,
            [this](const modelec_interfaces::msg::StratState::SharedPtr msg)
            {
                if (msg->state == modelec_interfaces::msg::StratState::STOP)
                {
                    RCLCPP_INFO(node_->get_logger(), "Game stop");
                    isGameStarted_ = false;
                }
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

        auto timer = new QTimer(this);
        connect(timer, &QTimer::timeout, this, [this]() {
            if (!isGameStarted_) return;
            auto elapsed = (std::chrono::system_clock::now().time_since_epoch().count() - start_time_) / 1e9;
            timer_label_->setText(QString("%1 s").arg(elapsed));
        });
        timer->start(1000);
    }

    void MapPage::AskMap()
    {
        reset_timer_ = node_->create_wall_timer(
            std::chrono::seconds(1),
            [this]()
            {
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

                ask_map_obstacle_client_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());

                reset_timer_->cancel();
            });
    }

    void MapPage::Reset()
    {
        isGameStarted_ = false;
        lastWapointWasEnd = true;
        score_ = 0;
        score_label_->setText(QString("Score: %1").arg(score_));
        timer_label_->setText("00");

        qpoints.clear();
        obstacle_.clear();

        AskMap();
    }

    void MapPage::paintEvent(QPaintEvent* paint_event)
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

    void MapPage::OnObstacleReceived(const modelec_interfaces::msg::Obstacle::SharedPtr msg)
    {
        obstacle_[msg->id] = *msg;
        obstacles_dirty_ = true;
        update();
    }

    void MapPage::resizeEvent(QResizeEvent* event)
    {
        QWidget::resizeEvent(event);

        ratioBetweenMapAndWidgetX_ = width() / 3000.0f;
        ratioBetweenMapAndWidgetY_ = height() / 2000.0f;

        bg_dirty_ = true;
        obstacles_dirty_ = true;
        waypoints_dirty_ = true;
        update();
    }

    void MapPage::updateBackgroundCache()
    {
        background_cache_ = QPixmap(size());
        background_cache_.fill(Qt::transparent);

        QPainter p(&background_cache_);
        renderer_->render(&p, rect());
        bg_dirty_ = false;
    }

    void MapPage::updateObstaclesCache()
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

    void MapPage::updateWaypointsCache()
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
