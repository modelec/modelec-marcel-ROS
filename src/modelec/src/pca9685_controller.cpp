#include "modelec/pca9685_controller.hpp"

namespace Modelec {

    PCA9685Controller::PCA9685Controller() : Node("pca9685_controller") {
        if (wiringPiSetup() == -1) {
            RCLCPP_ERROR(this->get_logger(), "WiringPi setup failed.");
            return;
        }

        // Initialize I2C communication with PCA9685
        fd = wiringPiI2CSetup(PCA9685_ADDR);
        if (fd == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize I2C communication with PCA9685.");
            return;
        }

        initializePCA9685();

        // Subscribe to topics for servo and solenoid control
        servo_subscriber_ = this->create_subscription<modelec_interface::msg::PCA9685Servo>(
            "servo_control", 10, [this](const modelec_interface::msg::PCA9685Servo::SharedPtr msg) {
                if (active_servos.find(msg->pin) == active_servos.end()) {
                    RCLCPP_ERROR(this->get_logger(), "Servo on pin %d is not active.", msg->pin);
                    return;
                }
                this->setServoPWM(msg->pin, msg->angle);
            });

        clear_subscriber_ = this->create_subscription<std_msgs::msg::Empty>(
            "clear_pca9685", 10, [this](const std_msgs::msg::Empty::SharedPtr) {
                this->clearAllDevices();
            });

        add_servo_service_ = this->create_service<modelec_interface::srv::NewServoMotor>(
            "add_servo", [this](const modelec_interface::srv::NewServoMotor::Request::SharedPtr request,
                                modelec_interface::srv::NewServoMotor::Response::SharedPtr response) {
                if (active_servos.find(request->pin) == active_servos.end()) {
                    active_servos.insert(request->pin);
                    response->success = true;
                } else {
                    response->success = false;
                }
            });

        // Declare frequency as a parameter and configure the PCA9685
        this->declare_parameter<int>("frequency", 50);
        frequency = this->get_parameter("frequency").as_int();
        configurePCA9685Frequency(frequency);

        // Handle dynamic parameter changes
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &parameters) {
                return onParameterChange(parameters);
            });

        clearAllDevices();
    }

    // Initialize PCA9685 in normal operation mode
    void PCA9685Controller::initializePCA9685() {
        writePCA9685Register(0x00, 0x00);
    }

    // Configure the PWM frequency
    void PCA9685Controller::configurePCA9685Frequency(int frequency) {
        int prescale = static_cast<int>(PCA9685_FREQUENCY / (PCA9685_RESOLUTION * frequency) - 1);
        writePCA9685Register(0x00, 0x10);  // Enter sleep mode
        writePCA9685Register(0xFE, prescale);  // Set prescale value
        writePCA9685Register(0x00, 0x00);  // Restart PCA9685
    }

    void PCA9685Controller::SetPCA9685PWM(int channel, int on_time, int off_time) {
        wiringPiI2CWriteReg8(fd, 0x06 + 4 * channel, on_time & 0xFF);
        wiringPiI2CWriteReg8(fd, 0x07 + 4 * channel, on_time >> 8);
        wiringPiI2CWriteReg8(fd, 0x08 + 4 * channel, off_time & 0xFF);
        wiringPiI2CWriteReg8(fd, 0x09 + 4 * channel, off_time >> 8);
    }

    // Set the servo angle on a specific channel
    void PCA9685Controller::setServoPWM(int channel, double angle) {
        int on_time = static_cast<int>(SERVO_MIN + (SERVO_MAX - SERVO_MIN) * angle / 180);
        SetPCA9685PWM(channel, 0, on_time);
    }

    // Clear all channels (reset devices)
    void PCA9685Controller::clearAllDevices() {
        for (int channel = 0; channel < 16; ++channel) {
            SetPCA9685PWM(channel, 0, 0);
        }
    }

    // Write to a specific PCA9685 register
    void PCA9685Controller::writePCA9685Register(uint8_t reg, uint8_t value) {
        if (wiringPiI2CWriteReg8(fd, reg, value) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write to register 0x%02X", reg);
        }
    }

    // Handle dynamic parameter changes
    rcl_interfaces::msg::SetParametersResult PCA9685Controller::onParameterChange(const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &parameter : parameters) {
            if (parameter.get_name() == "frequency") {
                updatePCA9685();
            }
        }

        return result;
    }

    // Update PCA9685 when frequency parameter changes
    void PCA9685Controller::updatePCA9685() {
        int new_frequency = this->get_parameter("frequency").as_int();
        if (new_frequency != frequency) {
            frequency = new_frequency;
            configurePCA9685Frequency(frequency);
        }
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Modelec::PCA9685Controller>());
    rclcpp::shutdown();
    return 0;
}