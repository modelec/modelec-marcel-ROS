#include "modelec/gamecontroller_listener.hpp"
#include "modelec/utils.hpp"

namespace Modelec {
    ControllerListener::ControllerListener() : Node("controller_listener")
    {
        // Subscribe to the 'joy' topic
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&ControllerListener::joy_callback, this, std::placeholders::_1));

        servo_publisher_ = this->create_publisher<ServoMode>("arm_control", 10);

        arduino_publisher_ = this->create_publisher<std_msgs::msg::String>("send_to_arduino", 10);

        clear_pca_publisher_ = this->create_publisher<std_msgs::msg::Empty>("clear_pca9685", 10);
    }

    void ControllerListener::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // to check mapping : https://index.ros.org//p/joy/
        // bon en fait la doc dit de la merde pour voir les bon checker directement en loggant le topic joy
        CheckButton(msg);
        CheckAxis(msg);
    }

    void ControllerListener::CheckButton(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->buttons[2] == 1) {
            if (button_2_was_pressed) {
                return;
            }
            auto message = ServoMode();
            message.pin = 0;
            if (pinces[0] == ServoMode::PINCE_CLOSED) {
                pinces[0] = ServoMode::PINCE_OPEN;
            } else {
                pinces[0] = ServoMode::PINCE_CLOSED;
            }
            message.mode = pinces[0];
            servo_publisher_->publish(message);
            button_2_was_pressed = true;
        } else {
            button_2_was_pressed = false;
        }
        if (msg->buttons[3] == 1) {
            if (button_3_was_pressed) {
                return;
            }
            auto message = ServoMode();
            message.pin = 1;
            if (pinces[1] == ServoMode::PINCE_CLOSED) {
                pinces[1] = ServoMode::PINCE_OPEN;
            } else {
                pinces[1] = ServoMode::PINCE_CLOSED;
            }
            message.mode = pinces[1];
            servo_publisher_->publish(message);
            button_3_was_pressed = true;
        } else {
            button_3_was_pressed = false;
        }
        if (msg->buttons[1] == 1) {
            if (button_1_was_pressed) {
                return;
            }
            auto message = ServoMode();
            message.pin = 2;
            if (pinces[2] == ServoMode::PINCE_CLOSED) {
                pinces[2] = ServoMode::PINCE_OPEN;
            } else {
                pinces[2] = ServoMode::PINCE_CLOSED;
            }
            message.mode = pinces[2];
            servo_publisher_->publish(message);
            button_1_was_pressed = true;
        } else {
            button_1_was_pressed = false;
        }

        // arm
        if (msg->buttons[0] == 1) {
            if (button_0_was_pressed) {
                return;
            }
            auto message = ServoMode();
            if (arm == ServoMode::ARM_BOTTOM) {
                arm = ServoMode::ARM_TOP;
            } else {
                arm = ServoMode::ARM_BOTTOM;
            }
            message.mode = arm;
            message.is_arm = true;
            servo_publisher_->publish(message);
            button_0_was_pressed = true;
        } else {
            button_0_was_pressed = false;
        }

        if (msg->buttons[4]) {
            if (button_4_was_pressed) {
                return;
            }
            auto message = std_msgs::msg::String();
            message.data = "W";
            arduino_publisher_->publish(message);
            button_4_was_pressed = true;
        } else {
            button_4_was_pressed = false;
        }

        if (msg->buttons[5]) {
            if (button_5_was_pressed) {
                return;
            }
            clear_pca_publisher_->publish(std_msgs::msg::Empty());
            button_5_was_pressed = true;
        } else {
            button_5_was_pressed = false;
        }
    }

    void ControllerListener::CheckAxis(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        auto message = std_msgs::msg::String();
        int speed = 0;
        if (msg->axes[1] < 0.1 && msg->axes[1] > -0.1) {
            speed = 0;
        } else {
            speed = static_cast<int>(Modelec::mapValue(static_cast<float>(msg->axes[1]), -1.0f, 1.0f, -310.0f, 310.0f));
        }

        if (speed != last_speed) {
            message.data = "V " + std::to_string(speed);
            arduino_publisher_->publish(message);
            last_speed = speed;
        }

        int rotation = 0;
        if (msg->axes[3] < 0.1 && msg->axes[3] > -0.1) {
            rotation = 0;
        } else {
            rotation = static_cast<int>(Modelec::mapValue(static_cast<float>(-msg->axes[3]), -1.0f, 1.0f, -310.0f, 310.0f));
        }

        if (rotation != last_rotation) {
            message.data = "R " + std::to_string(rotation);
            arduino_publisher_->publish(message);
            last_rotation = rotation;
        }

        if (msg->axes[2] != last_solar_1_angle) {
            int solarPannelAngle = static_cast<int>(Modelec::mapValue(static_cast<float>(msg->axes[2]), -1.0f, 1.0f, solarPannelServos[0].startAngle, solarPannelServos[0].endAngle));
            auto solarPannelAngleMessage = modelec_interface::msg::PCA9685Servo();
            solarPannelAngleMessage.pin = solarPannelServos[0].pin;
            solarPannelAngleMessage.angle = solarPannelAngle;
            pca9685_publisher_->publish(solarPannelAngleMessage);
            last_solar_1_angle = solarPannelAngle;
        }

        if (msg->axes[5] != last_solar_2_angle) {
            int solarPannelAngle = static_cast<int>(Modelec::mapValue(static_cast<float>(msg->axes[5]), -1.0f, 1.0f, solarPannelServos[1].endAngle, solarPannelServos[1].startAngle));
            auto solarPannelAngleMessage = modelec_interface::msg::PCA9685Servo();
            solarPannelAngleMessage.pin = solarPannelServos[1].pin;
            solarPannelAngleMessage.angle = solarPannelAngle;
            pca9685_publisher_->publish(solarPannelAngleMessage);
            last_solar_2_angle = solarPannelAngle;
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
