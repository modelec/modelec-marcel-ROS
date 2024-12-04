#pragma once

#include <rclcpp/rclcpp.hpp>
#include <unordered_set>
#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <modelec_interface/msg/pca9685_servo.hpp>
#include <modelec_interface/srv/new_servo_motor.hpp>
#include <std_msgs/msg/empty.hpp>

#define PCA9685_FREQUENCY 25000000.0
#define PCA9685_RESOLUTION 4096.0

// Servo and solenoid PWM limits
#define SERVO_MIN 82
#define SERVO_MAX 492
#define PCA9685_ADDR 0x40  // I2C address for the PCA9685

namespace Modelec {
    class PCA9685Controller : public rclcpp::Node {
    public:
        PCA9685Controller();

    private:
        int fd;  // File descriptor for I2C communication
        rclcpp::Subscription<modelec_interface::msg::PCA9685Servo>::SharedPtr servo_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr clear_subscriber_;
        int frequency = 50;  // Default PWM frequency (Hz)

        std::unordered_set<int> active_servos;

        // service to add a servo
        rclcpp::Service<modelec_interface::srv::NewServoMotor>::SharedPtr add_servo_service_;

        OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
        rcl_interfaces::msg::SetParametersResult onParameterChange(const std::vector<rclcpp::Parameter> &parameters);
        void updatePCA9685();

        // PCA9685 configuration and control
        void initializePCA9685();
        void configurePCA9685Frequency(int frequency);
        void SetPCA9685PWM(int channel, int on_time, int off_time);
        void setServoPWM(int channel, double angle);
        void clearAllDevices();
        void writePCA9685Register(uint8_t reg, uint8_t value);
    };
}
