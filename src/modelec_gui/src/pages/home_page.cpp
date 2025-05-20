#include <QPainter>
#include <modelec_gui/pages/home_page.hpp>

#include <QVBoxLayout>
#include <modelec_interfaces/msg/detail/servo_mode__builder.hpp>

#include "../../../modelec_utils/include/modelec_utils/config.hpp"

namespace ModelecGUI
{
    HomePage::HomePage(rclcpp::Node::SharedPtr node, QWidget* parent)
        : QWidget(parent), node_(node),
          renderer_(new QSvgRenderer(QString(":/img/playmat/2025_FINAL.svg"), this))
    {
        spawn_pub_ = node_->create_publisher<modelec_interfaces::msg::Spawn>("/strat/spawn", 10);

        auto w = Modelec::Config::get<int>("config.spawn.width_mm");
        auto h = Modelec::Config::get<int>("config.spawn.height_mm");

        spawn_sub_ = node_->create_subscription<modelec_interfaces::msg::Spawn>("/nav/spawn", 10,
            [this, w, h](const modelec_interfaces::msg::Spawn::SharedPtr msg)
            {
                auto ratioX = 1200 / 3000.0f;
                auto ratioY = 800 / 2000.0f;

                auto* button = new QPushButton(this);
                spawn_buttons_.push_back(button);

                button->setText(msg->team_id == 0 ? "Yellow" : "Blue");
                button->setStyleSheet(
                    msg->team_id == 0
                        ? "background-color: rgba(255, 255, 0, 128); border: none; color: black; font-size: 24px;"
                        : "background-color: rgba(0, 0, 255, 128); border: none; color: white; font-size: 24px;"
                );

                button->move(
                    static_cast<int>(msg->x * ratioX - (w * ratioX) / 2),
                    static_cast<int>(800 - msg->y * ratioY - (h * ratioY) / 2)
                );

                button->setFixedSize(w * ratioX, h * ratioY);

                button->show();

                connect(button, &QPushButton::clicked, this, [this, msg]()
                {
                    modelec_interfaces::msg::Spawn team_msg;
                    team_msg.team_id = msg->team_id;
                    team_msg.name = msg->name;
                    spawn_pub_->publish(team_msg);

                    emit TeamChoose();
                });
            });

        reset_strat_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/strat/reset", 10);

        ask_spawn_client_ = node_->create_client<std_srvs::srv::Empty>("/nav/ask_spawn");
        ask_spawn_client_->wait_for_service();
        auto ask_spawn_request_ = std::make_shared<std_srvs::srv::Empty::Request>();
        auto res = ask_spawn_client_->async_send_request(ask_spawn_request_);
        rclcpp::spin_until_future_complete(node_->get_node_base_interface(), res);
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
