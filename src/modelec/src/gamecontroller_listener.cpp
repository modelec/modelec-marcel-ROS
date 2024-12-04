#include "modelec/gamecontroller_listener.hpp"

namespace Modelec {
    ControllerListener::ControllerListener() : Node("controller_listener")
    {
        // Subscribe to the 'joy' topic
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&ControllerListener::joy_callback, this, std::placeholders::_1));

        servo_publisher_ = this->create_publisher<ServoMode>("arm_control", 10);

        arduino_publisher_ = this->create_publisher<std_msgs::msg::String>("send_to_arduino", 10);
    }

    void ControllerListener::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // Handle the joystick input here
        RCLCPP_INFO(this->get_logger(), "Joystick axes: [%f, %f, %f, %f, %f, %f]",
                    msg->axes[0], msg->axes[1], msg->axes[2], msg->axes[3], msg->axes[4], msg->axes[5]);

        RCLCPP_INFO(this->get_logger(), "Joystick buttons: [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d]",
                    msg->buttons[0], msg->buttons[1], msg->buttons[2], msg->buttons[3], 
                    msg->buttons[4], msg->buttons[5], msg->buttons[6], msg->buttons[7], 
                    msg->buttons[8], msg->buttons[9], msg->buttons[10], msg->buttons[11]);


        CheckButton(msg);
        CheckAxis(msg);
    }

    void ControllerListener::CheckButton(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->buttons[2] == 1) {
            auto message = ServoMode();
            message.pin = 0;
            if (pinces[0] == ServoMode::PINCE_CLOSED) {
                pinces[0] = ServoMode::PINCE_OPEN;
            } else {
                pinces[0] = ServoMode::PINCE_CLOSED;
            }
            message.mode = pinces[0];
            servo_publisher_->publish(message);
        }
        if (msg->buttons[3] == 1) {
            auto message = ServoMode();
            message.pin = 1;
            if (pinces[1] == ServoMode::PINCE_CLOSED) {
                pinces[1] = ServoMode::PINCE_OPEN;
            } else {
                pinces[1] = ServoMode::PINCE_CLOSED;
            }
            message.mode = pinces[1];
            servo_publisher_->publish(message);
        }
        if (msg->buttons[1] == 1) {
            auto message = ServoMode();
            message.pin = 2;
            if (pinces[2] == ServoMode::PINCE_CLOSED) {
                pinces[2] = ServoMode::PINCE_OPEN;
            } else {
                pinces[2] = ServoMode::PINCE_CLOSED;
            }
            message.mode = pinces[2];
            servo_publisher_->publish(message);
        }

        // arm
        if (msg->buttons[0] == 1) {
            auto message = ServoMode();
            if (arm == ServoMode::ARM_BOTTOM) {
                arm = ServoMode::ARM_TOP;
            } else if (arm == ServoMode::ARM_TOP) {
                arm = ServoMode::ARM_MIDDLE;
            } else {
                arm = ServoMode::ARM_BOTTOM;
            }
            message.mode = arm;
            servo_publisher_->publish(message);
        }

        if (msg->buttons[9] || msg->buttons[10]) {
            auto message = std_msgs::msg::String();
            message.data = "W\n";
            arduino_publisher_->publish(message);
        }
    }

    void ControllerListener::CheckAxis(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // arm
        // TODO
        // do not know the range of the axis value, like if it is between -1 and 1 or not
        auto message = std_msgs::msg::String();
        int speed = 0;
        if (msg->axes[1] < 0.1 && msg->axes[1] > -0.1) {
            speed = 0;
        }

        if (speed != last_speed) {
            message.data = "V " + std::to_string(speed) + "\n";
            arduino_publisher_->publish(message);
            last_speed = speed;
        }

        int rotation = 0;
        if (msg->axes[2] < 0.1 && msg->axes[2] > -0.1) {
            rotation = 0;
        }

        if (rotation != last_rotation) {
            message.data = "R " + std::to_string(rotation) + "\n";
            arduino_publisher_->publish(message);
            last_rotation = rotation;
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Modelec::ControllerListener>());
    rclcpp::shutdown();
    return 0;
}
