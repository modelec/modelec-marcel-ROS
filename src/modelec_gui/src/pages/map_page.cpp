#include <modelec_gui/pages/map_page.hpp>
#include <modelec_utils/utils.hpp>

#include <QMouseEvent>
#include <utility>

namespace ModelecGUI
{
    MapPage::MapPage(rclcpp::Node::SharedPtr node, QWidget* parent) : QWidget(parent), renderer(new QSvgRenderer(QString(":/img/playmat/grid_v1.svg"), this)), node_(std::move(node))
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

        add_waypoint_sub_ = node_->create_subscription<modelec_interfaces::msg::OdometryAddWaypoint>("odometry/add_waypoint", 10,
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

        go_to_pub_ = node_->create_publisher<modelec_interfaces::msg::OdometryPos>("nav/go_to", 10);
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
    }

    void MapPage::mousePressEvent(QMouseEvent* event)
    {
        QWidget::mousePressEvent(event);

        modelec_interfaces::msg::OdometryPos msg;
        msg.x = Modelec::mapValue(event->pos().x(), 0, width(), 0, 3000);
        msg.y = 2000 - Modelec::mapValue(event->pos().y(), 0, height(), 0, 2000);

        go_to_pub_->publish(msg);

        /*
        qpoints.push_back(event->pos());

        modelec_interfaces::msg::OdometryAddWaypoint msg;
        msg.x = Modelec::mapValue(event->pos().x(), 0, width(), 0, 3000);
        msg.y = 2000 - Modelec::mapValue(event->pos().y(), 0, height(), 0, 2000);
        msg.is_end = false;
        msg.id = points.size();

        if (points.empty())
        {
            QPointF vec = QPoint(msg.x, msg.y) - robotPosPoint;
            msg.theta = std::atan2(vec.y(), vec.x());
        }
        else
        {
            auto lastPoint = points.back();
            QPointF vec = QPoint(msg.x, msg.y) - QPoint(lastPoint.x, lastPoint.y);
            msg.theta = std::atan2(vec.y(), vec.x());
        }

        points.push_back(msg);

        update();*/
    }
}
