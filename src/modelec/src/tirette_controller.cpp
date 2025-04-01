#include <modelec/tirette_controller.hpp>
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

        tirette_state = false;

        // Initialize the service
        service = this->create_service<modelec_interface::srv::Tirette>(
            "tirette",
            std::bind(&TiretteController::check_tirette, this, std::placeholders::_1, std::placeholders::_2));

        // Initialize the publisher
        publisher = this->create_publisher<std_msgs::msg::Bool>("tirette_state", 10);

        // Initialize the timer
        timer = this->create_wall_timer(std::chrono::milliseconds(REFRESH_RATE), [this]() {
            // TODO : change that to publish in continue (so change the main program)
            bool lastState = tirette_state;
            tirette_state = digitalRead(GPIO_TIRETTE) == LOW;

            if (lastState == LOW && tirette_state == HIGH) {
                auto msg = std_msgs::msg::Bool();
                msg.data = tirette_state;
                publisher->publish(msg);
            }
        });
    }

    void TiretteController::check_tirette(const std::shared_ptr<TiretteInterface::Request>, std::shared_ptr<TiretteInterface::Response> response)
    {
        response->state = tirette_state;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Modelec::TiretteController>());
    rclcpp::shutdown();
    return 0;
}
