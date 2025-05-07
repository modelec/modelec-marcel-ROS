#include <QPainter>
#include <modelec_gui/pages/home_page.hpp>

#include <QVBoxLayout>

namespace ModelecGUI
{
    HomePage::HomePage(rclcpp::Node::SharedPtr node, QWidget* parent)
       : QWidget(parent), node_(node),
         renderer_(new QSvgRenderer(QString(":/img/playmat/2025_FINAL.svg"), this))
    {
        yellow_button_ = new QPushButton("Yellow", this);
        blue_button_ = new QPushButton("Blue", this);

        yellow_button_->setStyleSheet("background-color: rgba(255, 255, 0, 128); border: none; color: black; font-size: 24px;");
        blue_button_->setStyleSheet("background-color: rgba(0, 0, 255, 128); border: none; color: white; font-size: 24px;");

        yellow_button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        blue_button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        h_layout_ = new QHBoxLayout();
        h_layout_->setContentsMargins(0, 0, 0, 0);
        h_layout_->setSpacing(0);
        h_layout_->addWidget(yellow_button_);
        h_layout_->addWidget(blue_button_);
        h_layout_->setStretch(0, 1);
        h_layout_->setStretch(1, 1);

        v_layout_ = new QVBoxLayout();
        v_layout_->setContentsMargins(0, 0, 0, 0);
        v_layout_->setSpacing(0);
        v_layout_->addLayout(h_layout_, 1);

        setLayout(v_layout_);

        team_pub_ = node_->create_publisher<std_msgs::msg::Int8>("/strat/team", 10);

        reset_strat_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/strat/reset", 10);

        connect(yellow_button_, &QPushButton::clicked, this, &HomePage::onYellowButtonClicked);
        connect(blue_button_, &QPushButton::clicked, this, &HomePage::onBlueButtonClicked);
    }

    void HomePage::Init()
    {
        reset_strat_pub_->publish(std_msgs::msg::Empty());
    }

    void HomePage::onYellowButtonClicked()
    {
        std_msgs::msg::Int8 msg;
        msg.data = 0;
        team_pub_->publish(msg);

        emit TeamChoose();
    }

    void HomePage::onBlueButtonClicked()
    {
        std_msgs::msg::Int8 msg;
        msg.data = 1;
        team_pub_->publish(msg);

        emit TeamChoose();
    }

    void HomePage::paintEvent(QPaintEvent* paint_event)
    {
        QWidget::paintEvent(paint_event);

        QPainter painter(this);
        renderer_->render(&painter, rect()); // Scales SVG to widget size
    }
}
