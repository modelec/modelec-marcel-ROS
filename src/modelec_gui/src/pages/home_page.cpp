#include <QPainter>
#include <modelec_gui/pages/home_page.hpp>

#include <QVBoxLayout>
#include <modelec_utils/config.hpp>

namespace ModelecGUI
{
    HomePage::HomePage(rclcpp::Node::SharedPtr node, QWidget* parent)
        : QWidget(parent), node_(node),
          renderer_(new QSvgRenderer(QString(":/img/playmat/2026_FINAL.svg"), this))
    {
        spawn_pub_ = node_->create_publisher<modelec_interfaces::msg::Spawn>("strat/spawn", 10);

        auto* layout = new QHBoxLayout(this);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(0);

        yellow_spawn_buttons_ = new QPushButton("Yellow", this);
        blue_spawn_buttons_   = new QPushButton("Blue", this);

        yellow_spawn_buttons_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        blue_spawn_buttons_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        yellow_spawn_buttons_->setStyleSheet(
            "QPushButton {"
            "  background-color: rgba(255, 215, 0, 140);"
            "  border: none;"
            "  font-size: 32px;"
            "  font-weight: bold;"
            "  color: black;"
            "}"
            "QPushButton:hover {"
            "  background-color: rgba(255, 215, 0, 180);"
            "}"
        );

        blue_spawn_buttons_->setStyleSheet(
            "QPushButton {"
            "  background-color: rgba(0, 90, 200, 140);"
            "  border: none;"
            "  font-size: 32px;"
            "  font-weight: bold;"
            "  color: white;"
            "}"
            "QPushButton:hover {"
            "  background-color: rgba(0, 90, 200, 180);"
            "}"
        );

        layout->addWidget(yellow_spawn_buttons_, 1);
        layout->addWidget(blue_spawn_buttons_, 1);

        connect(yellow_spawn_buttons_, &QPushButton::clicked, this, [this]()
        {
            modelec_interfaces::msg::Spawn msg;
            msg.team_id = 0;
            msg.name = modelec_interfaces::msg::Spawn::TOP;
            spawn_pub_->publish(msg);
            emit TeamChoose();
        });

        connect(blue_spawn_buttons_, &QPushButton::clicked, this, [this]()
        {
            modelec_interfaces::msg::Spawn msg;
            msg.team_id = 1;
            msg.name = modelec_interfaces::msg::Spawn::TOP;
            spawn_pub_->publish(msg);
            emit TeamChoose();
        });

        reset_strat_pub_ = node_->create_publisher<std_msgs::msg::Empty>("strat/reset", 10);
    }

    void HomePage::Init()
    {
        reset_strat_pub_->publish(std_msgs::msg::Empty());
    }

    void HomePage::paintEvent(QPaintEvent* paint_event)
    {
        QWidget::paintEvent(paint_event);

        QPainter painter(this);
        renderer_->render(&painter, rect()); // Scales SVG to widget size
    }
}
