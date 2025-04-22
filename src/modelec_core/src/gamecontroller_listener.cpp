#include "modelec/gamecontroller_listener.hpp"
#include "modelec_utils/utils.hpp"
#include <modelec_interfaces/srv/add_serial_listener.hpp>

namespace Modelec
{
    ControllerListener::ControllerListener() : Node("controller_listener")
    {
        // Subscribe to the 'joy' topic
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&ControllerListener::joy_callback, this, std::placeholders::_1));

        servo_publisher_ = this->create_publisher<ServoMode>("arm_control", 10);

        // Service to create a new serial listener
        auto request = std::make_shared<modelec_interfaces::srv::AddSerialListener::Request>();
        request->name = "odometry";
        request->bauds = 115200;
        request->serial_port = "/dev/ttyACM0";
        auto client = this->create_client<modelec_interfaces::srv::AddSerialListener>("add_serial_listener");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
        auto result = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            if (auto res = result.get())
            {
                if (res->success)
                {
                    RCLCPP_INFO(this->get_logger(), "Serial listener created");

                    odometry_publisher_ = this->create_publisher<std_msgs::msg::String>(result.get()->subscriber, 10);

                    clear_pca_publisher_ = this->create_publisher<std_msgs::msg::Empty>("clear_pca9685", 10);

                    pca9685_publisher_ = this->create_publisher<modelec_interfaces::msg::PCA9685Servo>(
                        "servo_control", 10);
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to create serial listener");
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to get result from service");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }

        client_ = this->create_client<modelec_interfaces::srv::AddServoMotor>("add_servo");

        // ensure the server is up
        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        for (auto servo : solarPannelServos)
        {
            auto request = std::make_shared<modelec_interfaces::srv::AddServoMotor::Request>();
            request->pin = servo.pin;
            auto res = client_->async_send_request(request);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), res) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                if (!res.get()->success)
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to add servo on pin %d", servo.pin);
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Added servo on pin %d", servo.pin);
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Service call failed");
            }
        }
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
        if (msg->buttons[2] == 1)
        {
            if (button_2_was_pressed)
            {
                return;
            }
            auto message = ServoMode();
            message.pin = 0;
            if (pinces[0] == ServoMode::PINCE_CLOSED)
            {
                pinces[0] = ServoMode::PINCE_OPEN;
            }
            else
            {
                pinces[0] = ServoMode::PINCE_CLOSED;
            }
            message.mode = pinces[0];
            servo_publisher_->publish(message);
            button_2_was_pressed = true;
        }
        else
        {
            button_2_was_pressed = false;
        }
        if (msg->buttons[3] == 1)
        {
            if (button_3_was_pressed)
            {
                return;
            }
            auto message = ServoMode();
            message.pin = 1;
            if (pinces[1] == ServoMode::PINCE_CLOSED)
            {
                pinces[1] = ServoMode::PINCE_OPEN;
            }
            else
            {
                pinces[1] = ServoMode::PINCE_CLOSED;
            }
            message.mode = pinces[1];
            servo_publisher_->publish(message);
            button_3_was_pressed = true;
        }
        else
        {
            button_3_was_pressed = false;
        }
        if (msg->buttons[1] == 1)
        {
            if (button_1_was_pressed)
            {
                return;
            }
            auto message = ServoMode();
            message.pin = 2;
            if (pinces[2] == ServoMode::PINCE_CLOSED)
            {
                pinces[2] = ServoMode::PINCE_OPEN;
            }
            else
            {
                pinces[2] = ServoMode::PINCE_CLOSED;
            }
            message.mode = pinces[2];
            servo_publisher_->publish(message);
            button_1_was_pressed = true;
        }
        else
        {
            button_1_was_pressed = false;
        }

        // arm
        if (msg->buttons[0] == 1)
        {
            if (button_0_was_pressed)
            {
                return;
            }
            auto message = ServoMode();
            if (arm == ServoMode::ARM_BOTTOM)
            {
                arm = ServoMode::ARM_TOP;
            }
            else
            {
                arm = ServoMode::ARM_BOTTOM;
            }
            message.mode = arm;
            message.is_arm = true;
            servo_publisher_->publish(message);
            button_0_was_pressed = true;
        }
        else
        {
            button_0_was_pressed = false;
        }

        if (msg->buttons[4])
        {
            if (button_4_was_pressed)
            {
                return;
            }
            auto message = std_msgs::msg::String();
            message.data = "W";
            odometry_publisher_->publish(message);
            button_4_was_pressed = true;
        }
        else
        {
            button_4_was_pressed = false;
        }

        if (msg->buttons[5])
        {
            if (button_5_was_pressed)
            {
                return;
            }
            clear_pca_publisher_->publish(std_msgs::msg::Empty());
            button_5_was_pressed = true;
        }
        else
        {
            button_5_was_pressed = false;
        }
    }

    void ControllerListener::CheckAxis(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        auto message = std_msgs::msg::String();
        int speed = 0;
        if (msg->axes[1] < 0.1 && msg->axes[1] > -0.1)
        {
            speed = 0;
        }
        else
        {
            speed = static_cast<int>(Modelec::mapValue(static_cast<float>(msg->axes[1]), -1.0f, 1.0f, -310.0f, 310.0f));
        }

        if (speed != last_speed)
        {
            message.data = "V " + std::to_string(speed);
            odometry_publisher_->publish(message);
            last_speed = speed;
        }

        int rotation = 0;
        if (msg->axes[3] < 0.1 && msg->axes[3] > -0.1)
        {
            rotation = 0;
        }
        else
        {
            rotation = static_cast<int>(Modelec::mapValue(static_cast<float>(-msg->axes[3]), -1.0f, 1.0f, -310.0f,
                                                          310.0f));
        }

        if (rotation != last_rotation)
        {
            message.data = "R " + std::to_string(rotation);
            odometry_publisher_->publish(message);
            last_rotation = rotation;
        }

        if (msg->axes[2] != last_solar_1_angle)
        {
            int solarPannelAngle = static_cast<int>(Modelec::mapValue(static_cast<float>(msg->axes[2]), -1.0f, 1.0f,
                                                                      solarPannelServos[0].startAngle,
                                                                      solarPannelServos[0].endAngle));
            auto solarPannelAngleMessage = modelec_interfaces::msg::PCA9685Servo();
            solarPannelAngleMessage.pin = solarPannelServos[0].pin;
            solarPannelAngleMessage.angle = solarPannelAngle;
            pca9685_publisher_->publish(solarPannelAngleMessage);
            last_solar_1_angle = msg->axes[2];
        }

        if (msg->axes[5] != last_solar_2_angle)
        {
            int solarPannelAngle = static_cast<int>(Modelec::mapValue(static_cast<float>(msg->axes[5]), -1.0f, 1.0f,
                                                                      solarPannelServos[1].startAngle,
                                                                      solarPannelServos[1].endAngle));
            auto solarPannelAngleMessage = modelec_interfaces::msg::PCA9685Servo();
            solarPannelAngleMessage.pin = solarPannelServos[1].pin;
            solarPannelAngleMessage.angle = solarPannelAngle;
            pca9685_publisher_->publish(solarPannelAngleMessage);
            last_solar_2_angle = msg->axes[5];
        }
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Modelec::ControllerListener>());
    rclcpp::shutdown();
    return 0;
}
