#include <modelec/move_controller.hpp>

namespace Modelec {
    MoveController::MoveController() : Node("move_controller")
    {
        // Initialize the speed
        speedX = 0.0;
        speedZ = 0.0;

        // Initialize the target position
        x_target = 0.0;
        y_target = 0.0;
        z_target = 0.0;
        theta_target = 0.0;

        // Initialize the publisher
        publisher = this->create_publisher<std_msgs::msg::String>("robot_position", 10);

        // Initialize the subscriber
        subscriber = this->create_subscription<std_msgs::msg::String>(
            "move_target", 10,
            std::bind(&MoveController::move_target_callback, this, std::placeholders::_1));

        // Initialize the timer
        timer = this->create_wall_timer(std::chrono::milliseconds(REFRESH_RATE), std::bind(&MoveController::move, this));
    }

    void MoveController::move()
    {
        // Move the robot
        x += speedX;
        z += speedZ;

        // Check if the robot has reached the target position
        if ((x >= x_target && x+speedX < x_target) || (x <= x_target && x+speedX > x_target))
        {
            speedX = 0.0;
        }

        if ((z >= z_target && z+speedZ < z_target) || (z <= z_target && z+speedZ > z_target))
        {
            speedZ = 0.0;
        }

        // Prepare and publish the message
        auto msg = std_msgs::msg::String();
        msg.data = std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z) + " " + std::to_string(theta);
        publisher->publish(msg);
    }

    void MoveController::move_target_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        // Parse the target position
        std::istringstream iss(msg->data);
        iss >> x_target >> y_target >> z_target >> theta_target;

        // Calculate the speed
        speedX = (x_target - x) / 100;
        speedZ = (z_target - z) / 100;
        RCLCPP_INFO(this->get_logger(), "Target position: %f %f %f %f", x_target, y_target, z_target, theta_target);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Modelec::MoveController>());
    rclcpp::shutdown();
    return 0;
}