#include "modelec_interfaces/msg/servo_mode.hpp"
#include <modelec/arm_controller.hpp>

namespace Modelec {
    ArmController::ArmController() : Node("pince_controller") {
        this->servo_spublisher_ = this->create_publisher<modelec_interfaces::msg::PCA9685Servo>("servo_control", 10);
        client_ = this->create_client<modelec_interfaces::srv::AddServoMotor>("add_servo");

        // ensure the server is up
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        for (int pin : {PINCE_1_PIN, PINCE_2_PIN, PINCE_3_PIN, ARM_1_PIN, ARM_2_PIN}) {
            auto request = std::make_shared<modelec_interfaces::srv::AddServoMotor::Request>();
            request->pin = pin;
            auto res = client_->async_send_request(request);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), res) == rclcpp::FutureReturnCode::SUCCESS) {
                if (!res.get()->success) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to add servo on pin %d", pin);
                } else {
                    RCLCPP_INFO(this->get_logger(), "Added servo on pin %d", pin);
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Service call failed");
            }
        }

        subscription_ = this->create_subscription<modelec_interfaces::msg::ServoMode>(
            "arm_control", 10, [this](const modelec_interfaces::msg::ServoMode::SharedPtr msg) {
                if (msg->is_arm) {
                    ControlArm(msg);
                } else if (pince_pins.find(msg->pin) != pince_pins.end()) {
                    ControlPince(msg);
                }
            });
    }

    void ArmController::ControlPince(const Mode::SharedPtr msg) {
        auto pince = pince_pins[msg->pin];
        if (msg->mode == pince.mode) {
            RCLCPP_INFO(this->get_logger(), "Pince %d already in mode %d, no action required", msg->pin, msg->mode);
            return;
        }

        auto message = modelec_interfaces::msg::PCA9685Servo();
        message.pin = msg->pin;
        message.angle = pince.angles[msg->mode];
        servo_spublisher_->publish(message);

        pince_pins[msg->pin].mode = msg->mode;
    }

    void ArmController::ControlArm(const Mode::SharedPtr msg) {
        if (arm.mode == msg->mode) {
            return;
        }

        for (auto pince : pince_pins) {
            if (pince.second.mode != Mode::PINCE_CLOSED) {
                auto message = modelec_interfaces::msg::PCA9685Servo();
                message.pin = pince.second.pin;
                message.angle = pince.second.angles[msg->mode];
                servo_spublisher_->publish(message);
            }
        }

/*
        int direction = -1;
        if (arm.mode == Mode::ARM_BOTTOM && msg->mode == Mode::ARM_TOP) {
            direction = 1;
        } else if (arm.mode == Mode::ARM_TOP && msg->mode == Mode::ARM_BOTTOM) {
            direction = -1;
        } else if (arm.mode == Mode::ARM_MIDDLE && msg->mode == Mode::ARM_TOP) {
            direction = 1;
        } else if (arm.mode == Mode::ARM_TOP && msg->mode == Mode::ARM_MIDDLE) {
            direction = -1;
        }

        int startAngle = arm.pins[ARM_1_PIN][arm.mode];
        for (int angle = startAngle; angle <= arm.pins[ARM_1_PIN][msg->mode]; angle += 2 * direction) {
            auto message = modelec_interfaces::msg::PCA9685Servo();
            message.pin = ARM_1_PIN;
            message.angle = angle;
            publisher_->publish(message);

            message.pin = ARM_2_PIN;
            message.angle = 180 - angle;
            publisher_->publish(message);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        auto message = modelec_interfaces::msg::PCA9685Servo();
        message.pin = ARM_1_PIN;
        message.angle = arm.pins[ARM_1_PIN][msg->mode];
        publisher_->publish(message);

        message.pin = ARM_2_PIN;
        message.angle = arm.pins[ARM_2_PIN][msg->mode];
        publisher_->publish(message);*/
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Modelec::ArmController>());
    rclcpp::shutdown();
    return 0;
}
