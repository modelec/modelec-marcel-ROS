#include <modelec_gui/pages/map_page.hpp>
#include <modelec_utils/utils.hpp>

#include <QMouseEvent>
#include <utility>
#include <modelec_utils/config.hpp>
#include <cmath>

namespace ModelecGUI
{
    MapPage::MapPage(rclcpp::Node::SharedPtr node, QWidget* parent) : QWidget(parent), renderer(new QSvgRenderer(QString(":/img/playmat/2025_FINAL.svg"), this)), node_(node)
    {
        ratioBetweenMapAndWidgetX_ = width() / 3000.0f;
        ratioBetweenMapAndWidgetY_ = height() / 2000.0f;

        v_layout = new QVBoxLayout(this);

        timer_label_ = new QLabel("00", this);
        timer_label_->setAlignment(Qt::AlignCenter);
        timer_label_->setFont(QFont("Arial", 24));
        timer_label_->setStyleSheet("QLabel { color: white; }");
        score_label_ = new QLabel("Score: 0", this);
        score_label_->setAlignment(Qt::AlignCenter);
        score_label_->setFont(QFont("Arial", 24));
        score_label_->setStyleSheet("QLabel { color: white; }");

        h_layout = new QHBoxLayout(this);
        h_layout->addStretch();
        h_layout->addStretch();
        h_layout->addWidget(score_label_);
        h_layout->addStretch();
        h_layout->addWidget(timer_label_);
        h_layout->addStretch();
        h_layout->addStretch();

        v_layout->addLayout(h_layout);
        v_layout->addStretch();

        this->setLayout(v_layout);

        qpoints = {};

        robot_length_ = Modelec::Config::get<int>("config.robot.size.length_mm", 200);
        robot_width_ = Modelec::Config::get<int>("config.robot.size.width_mm", 300);

        enemy_length_ = Modelec::Config::get<int>("config.enemy.size.length_mm", 300);
        enemy_width_ = Modelec::Config::get<int>("config.enemy.size.width_mm", 300);

        add_waypoint_sub_ = node_->create_subscription<modelec_interfaces::msg::OdometryAddWaypoint>("odometry/add_waypoint", 100,
            [this](const modelec_interfaces::msg::OdometryAddWaypoint::SharedPtr msg) {
                if (lastWapointWasEnd)
                {
                    qpoints.clear();
                    lastWapointWasEnd = false;

                    qpoints.push_back(QPoint(robotPos.x * ratioBetweenMapAndWidgetX_, height() - robotPos.y * ratioBetweenMapAndWidgetY_));
                }

                if (msg->is_end)
                {
                    lastWapointWasEnd = true;
                }

                qpoints.push_back(QPoint(msg->x * ratioBetweenMapAndWidgetX_, height() - msg->y * ratioBetweenMapAndWidgetY_));
                update();
        });

        odometry_sub_ = node_->create_subscription<modelec_interfaces::msg::OdometryPos>("odometry/position", 10,
            [this](const modelec_interfaces::msg::OdometryPos::SharedPtr msg) {
                robotPos = *msg;
                update();
        });

        score_sub_ = node_->create_subscription<std_msgs::msg::Int64>("strat/score", 10,
            [this](const std_msgs::msg::Int64::SharedPtr msg) {
                score_+= msg->data;
                score_label_->setText(QString("Score: %1").arg(score_));
        });

        obstacle_added_sub_ = node_->create_subscription<modelec_interfaces::msg::Obstacle>("nav/obstacle/added", 40,
            [this](const modelec_interfaces::msg::Obstacle::SharedPtr msg) {
                OnObstacleReceived(msg);
        });

        obstacle_removed_sub_ = node_->create_subscription<modelec_interfaces::msg::Obstacle>("nav/obstacle/removed", 40,
            [this](const modelec_interfaces::msg::Obstacle::SharedPtr msg) {
                obstacle_.erase(msg->id);
        });

        enemy_pos_sub_ = node_->create_subscription<modelec_interfaces::msg::OdometryPos>("enemy/position", 10,
            [this](const modelec_interfaces::msg::OdometryPos::SharedPtr msg)
            {
                if (!hasEnemy) hasEnemy = true;

                enemy_pos_ = *msg;
                update();
            });

        strat_start_sub_ = node_->create_subscription<std_msgs::msg::Int64>("/strat/start_time", 10,
            [this](const std_msgs::msg::Int64::SharedPtr msg){
                isGameStarted_ = true;
                start_time_ = msg->data;
        });

        strat_state_sub_ = node_->create_subscription<modelec_interfaces::msg::StratState>("/strat/state", 10,
            [this](const modelec_interfaces::msg::StratState::SharedPtr msg){
                if (msg->state == modelec_interfaces::msg::StratState::STOP)
                {
                    RCLCPP_INFO(node_->get_logger(), "Game stop");
                    isGameStarted_ = false;
                }
        });

        // client to nav/map
        map_client_ = node_->create_client<modelec_interfaces::srv::MapSize>("nav/map_size");
        while (!map_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
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
                RCLCPP_INFO(node_->get_logger(), "Map received: %d x %d", res->width, res->height);
                map_width_ = res->width;
                map_height_ = res->height;
            }
        }

        ask_map_obstacle_client_ = node_->create_client<std_srvs::srv::Empty>("nav/ask_map_obstacle");
        while (!ask_map_obstacle_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(node_->get_logger(), "Waiting for the service...");
        }

        auto result2 = ask_map_obstacle_client_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
        rclcpp::spin_until_future_complete(node_->get_node_base_interface(), result2);
    }

    void MapPage::AskMap()
    {
        reset_timer_ = node_->create_wall_timer(
        std::chrono::seconds(1),
        [this]() {

            ask_map_obstacle_client_ = node_->create_client<std_srvs::srv::Empty>("nav/ask_map_obstacle");
            while (!ask_map_obstacle_client_->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
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

        qpoints.clear();

        AskMap();
    }

    void MapPage::paintEvent(QPaintEvent* paint_event)
    {
        QWidget::paintEvent(paint_event);

        if (isGameStarted_)
        {
            auto now = std::chrono::system_clock::now().time_since_epoch();
            auto start = std::chrono::nanoseconds(start_time_);
            auto elapsed = now - start;
            auto elapsed_s = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();
            timer_label_->setText(QString::number(elapsed_s));
        }

        QPainter painter(this);
        renderer->render(&painter, rect()); // Scales SVG to widget size
        painter.save();

        painter.setRenderHint(QPainter::Antialiasing);

        // --- Draw lines ---
        painter.setPen(QPen(Qt::red, 2)); // Red lines, 2px wide
        for (size_t i = 0; i + 1 < qpoints.size(); ++i) {
            painter.drawLine(qpoints[i], qpoints[i + 1]);
        }

        painter.setPen(Qt::NoPen);

        // --- Draw colored points ---
        const int radius = 5;

        for (size_t i = 0; i < qpoints.size(); ++i) {
            if (i == qpoints.size() - 1)
                painter.setBrush(Qt::blue);  // Last = blue
            else
                painter.setBrush(Qt::red);   // Middle = red

            painter.drawEllipse(qpoints[i], radius, radius);
        }

        painter.restore();

        if (show_obstacle_)
        {
            for (auto & [index, obs] : obstacle_)
            {
                painter.save();

                QPoint obsPoint(obs.x * ratioBetweenMapAndWidgetX_, height() - obs.y * ratioBetweenMapAndWidgetY_);
                painter.translate(obsPoint);
                painter.rotate(90 - obs.theta * (180.0 / M_PI));
                painter.setBrush(Qt::black);

                QRect toDraw(-(obs.width * ratioBetweenMapAndWidgetX_ / 2), -(obs.height * ratioBetweenMapAndWidgetY_ / 2),
                       obs.width * ratioBetweenMapAndWidgetX_, obs.height * ratioBetweenMapAndWidgetY_);

                painter.drawRect(toDraw);

                painter.restore();
            }

            // -- Draw enemy position --
            if (hasEnemy)
            {
                painter.setBrush(Qt::red);
                painter.drawRect((enemy_pos_.x - (enemy_width_ / 2)) * ratioBetweenMapAndWidgetX_, height() - (enemy_pos_.y + (enemy_length_ / 2)) * ratioBetweenMapAndWidgetY_, enemy_width_*ratioBetweenMapAndWidgetX_, enemy_length_*ratioBetweenMapAndWidgetY_);
            }
        }

        // -- Draw robot position --
        painter.translate(robotPos.x * ratioBetweenMapAndWidgetX_, height() - robotPos.y * ratioBetweenMapAndWidgetY_);
        painter.rotate(90 - robotPos.theta * (180.0 / M_PI));

        QRect rect(-(robot_width_ * ratioBetweenMapAndWidgetX_ / 2), -(robot_length_ * ratioBetweenMapAndWidgetY_ / 2),
                   robot_width_ * ratioBetweenMapAndWidgetX_, robot_length_ * ratioBetweenMapAndWidgetY_);
        painter.setBrush(Qt::green);
        painter.drawRect(rect);
    }

    void MapPage::OnObstacleReceived(const modelec_interfaces::msg::Obstacle::SharedPtr msg)
    {
        obstacle_[msg->id] = *msg;
    }

    void MapPage::resizeEvent(QResizeEvent* event)
    {
        QWidget::resizeEvent(event);

        ratioBetweenMapAndWidgetX_ = width() / 3000.0f;
        ratioBetweenMapAndWidgetY_ = height() / 2000.0f;
    }
}