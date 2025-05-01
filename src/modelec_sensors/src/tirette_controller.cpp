#include <modelec_sensors/tirette_controller.hpp>
#include <wiringPi.h>

namespace Modelec {
    TiretteController::TiretteController() : Node("tirette_controller")
    {
        if (wiringPiSetupGpio() == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize GPIO.");
            rclcpp::shutdown();
        }

        pinMode(GPIO_TIRETTE, INPUT);

        pullUpDnControl(GPIO_TIRETTE, PUD_UP);

        tirette_state_ = 0;

        // Initialize the service
        service_ = this->create_service<modelec_interfaces::srv::Tirette>(
            "tirette",
            std::bind(&TiretteController::check_tirette, this, std::placeholders::_1, std::placeholders::_2));

        // Initialize the publisher
        publisher_ = this->create_publisher<std_msgs::msg::Bool>("tirette", 10);
        publisher_change_ = this->create_publisher<std_msgs::msg::Bool>("tirette/continue", 10);

        // Initialize the timer
        timer_ = this->create_wall_timer(std::chrono::milliseconds(REFRESH_RATE), [this]() {
            // TODO : CHECK THAT
            bool lastState = tirette_state_;
            tirette_state_ = digitalRead(GPIO_TIRETTE);

            auto msg = std_msgs::msg::Bool();
            msg.data = tirette_state_;

            if (lastState == LOW && tirette_state_ == HIGH) {
                publisher_->publish(msg);
            }

            publisher_change_->publish(msg);
        });
    }

    void TiretteController::check_tirette(const std::shared_ptr<TiretteInterface::Request>, std::shared_ptr<TiretteInterface::Response> response)
    {
        response->state = tirette_state_;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Modelec::TiretteController>());
    rclcpp::shutdown();
    return 0;
}
