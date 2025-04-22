#include <modelec/move_controller.hpp>

namespace Modelec {
    MoveController::MoveController() : Node("move_controller")
    {
        // Initialize the publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("robot_position", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(REFRESH_RATE),
            std::bind(&MoveController::PublishPosition, this)
        );
    }

    void MoveController::PublishPosition()
    {
        std_msgs::msg::String msg;
        msg.data = std::to_string(x) + ";" + std::to_string(y) + ";" + std::to_string(theta);
        publisher_->publish(msg);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Modelec::MoveController>());
    rclcpp::shutdown();
    return 0;
}