#include <modelec_gui/pages/map_page.hpp>
#include <modelec_utils/utils.hpp>

#include <QMouseEvent>
#include <utility>

namespace ModelecGUI
{
    MapPage::MapPage(rclcpp::Node::SharedPtr node, QWidget* parent) : QWidget(parent), renderer(new QSvgRenderer(QString(":/img/playmat/2025_FINAL.svg"), this)), node_(node)
    {
        v_layout = new QVBoxLayout(this);
        v_layout->addStretch();

        h_layout = new QHBoxLayout(this);
        h_layout->addStretch();
        h_layout->addStretch();

        v_layout->addLayout(h_layout);

        this->setLayout(v_layout);

        qpoints = {};
        points = {};

        enemy_pos_.x = 1775;
        enemy_pos_.y = 200;
        enemy_pos_.theta = 3.1415/2;

        add_waypoint_sub_ = node_->create_subscription<modelec_interfaces::msg::OdometryAddWaypoint>("odometry/add_waypoint", 100,
            [this](const modelec_interfaces::msg::OdometryAddWaypoint::SharedPtr msg) {
                if (lastWapointWasEnd)
                {
                    qpoints.clear();
                    points.clear();
                    lastWapointWasEnd = false;
                }

                if (msg->is_end)
                {
                    lastWapointWasEnd = true;
                }

                qpoints.push_back(QPoint(Modelec::mapValue(static_cast<int>(msg->x), 0, 3000, 0, width()),
                                          height() - Modelec::mapValue(static_cast<int>(msg->y), 0, 2000, 0, height())));
                points.push_back(*msg);
                update();
        });

        // lambda
        odometry_sub_ = node_->create_subscription<modelec_interfaces::msg::OdometryPos>("odometry/position", 10,
            [this](const modelec_interfaces::msg::OdometryPos::SharedPtr msg) {
                robotPosPoint.setX(Modelec::mapValue(static_cast<int>(msg->x), 0, 3000, 0, width()));
                robotPosPoint.setY(height() - Modelec::mapValue(static_cast<int>(msg->y), 0, 2000, 0, height()));
                update();
        });

        obstacle_sub_ = node_->create_subscription<modelec_interfaces::msg::Obstacle>("nav/obstacle", 40,
            [this](const modelec_interfaces::msg::Obstacle::SharedPtr msg) {
                OnObstacleReceived(msg);
        });

        go_to_pub_ = node_->create_publisher<modelec_interfaces::msg::OdometryPos>("nav/go_to", 10);

        enemy_pos_pub_ = node_->create_publisher<modelec_interfaces::msg::OdometryPos>("enemy/position", 10);

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
                /* TODO - obstacle
                 * idea
                 * send only the size of the map
                 * and then send the obstacle with id throw the topic obstacle
                 * problem : if not initialized, idk what to do
                 * maybe solution ask to send back the obstacle throw an other service
                 */
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

    void MapPage::setPlaymatGrid()
    {
        renderer->load(QString(":/img/playmat/grid_v1.svg"));
        update();
    }

    void MapPage::setPlaymatMap()
    {
        renderer->load(QString(":/img/playmat/2025_FINAL.svg"));
        update();
    }

    void MapPage::toggleShowObstacle()
    {
        show_obstacle_ = !show_obstacle_;
    }

    void MapPage::paintEvent(QPaintEvent* paint_event)
    {
        QWidget::paintEvent(paint_event);

        QPainter painter(this);
        renderer->render(&painter, rect()); // Scales SVG to widget size

        painter.setRenderHint(QPainter::Antialiasing);

        // --- Draw lines ---
        painter.setPen(QPen(Qt::red, 2)); // Red lines, 2px wide
        if (!points.empty())
            painter.drawLine(robotPosPoint, qpoints[0]);

        for (size_t i = 0; i + 1 < points.size(); ++i) {
            painter.drawLine(qpoints[i], qpoints[i + 1]);
        }

        // --- Draw colored points ---
        const int radius = 5;

        painter.setBrush(Qt::green);
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(robotPosPoint, radius, radius); // Robot position

        for (size_t i = 0; i < points.size(); ++i) {
            if (i == points.size() - 1)
                painter.setBrush(Qt::blue);  // Last = blue
            else
                painter.setBrush(Qt::red);   // Middle = red

            painter.setPen(Qt::NoPen);
            painter.drawEllipse(qpoints[i], radius, radius);
        }

        if (show_obstacle_)
        {
            float ratioBetweenMapAndWidget = height() / 2000.0f;
            for (auto & [index, obs] : obstacle_)
            {
                painter.setBrush(Qt::black);
                painter.drawRect(obs.x * ratioBetweenMapAndWidget, height() - (obs.y + obs.height) * ratioBetweenMapAndWidget, obs.width * ratioBetweenMapAndWidget, obs.height * ratioBetweenMapAndWidget);
            }

            painter.setBrush(Qt::red);
            painter.drawRect((enemy_pos_.x - 150.0f) * ratioBetweenMapAndWidget, height() - (enemy_pos_.y + 150.0f) * ratioBetweenMapAndWidget, 300.0f*ratioBetweenMapAndWidget, 300.0f*ratioBetweenMapAndWidget);
        }
    }

    void MapPage::mousePressEvent(QMouseEvent* event)
    {
        QWidget::mousePressEvent(event);

        if (event->button() == Qt::LeftButton)
        {
            modelec_interfaces::msg::OdometryPos msg;
            msg.x = Modelec::mapValue(event->pos().x(), 0, width(), 0, 3000);
            msg.y = 2000 - Modelec::mapValue(event->pos().y(), 0, height(), 0, 2000);
            msg.theta = 0;

            go_to_pub_->publish(msg);
        }
        else if (event->button() == Qt::RightButton)
        {
            enemy_pos_.x = Modelec::mapValue(event->pos().x(), 0, width(), 0, 3000);
            enemy_pos_.y = 2000 - Modelec::mapValue(event->pos().y(), 0, height(), 0, 2000);
            enemy_pos_.theta = 0;

            enemy_pos_pub_->publish(enemy_pos_);
        }
    }

    void MapPage::OnObstacleReceived(const modelec_interfaces::msg::Obstacle::SharedPtr msg)
    {
        obstacle_.emplace(msg->id, *msg);
    }
}
