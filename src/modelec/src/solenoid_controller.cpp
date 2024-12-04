#include <modelec/solenoid_controller.hpp>
#include <wiringPi.h>

namespace Modelec {
    SolenoidController::SolenoidController() : Node("solenoid_controller") {
        wiringPiSetup();

        solenoid_subscriber_ = this->create_subscription<modelec_interface::msg::Solenoid>(
            "solenoid", 10, std::bind(&SolenoidController::activateSolenoid, this, std::placeholders::_1));

        solenoid_add_subscriber_ = this->create_subscription<modelec_interface::msg::Solenoid>(
            "add_solenoid", 10, std::bind(&SolenoidController::addSolenoidPin, this, std::placeholders::_1));
    }

    void SolenoidController::activateSolenoid(const modelec_interface::msg::Solenoid::SharedPtr msg) {
        // Activate solenoid
        if (solenoid_pin_.find(msg->pin) != solenoid_pin_.end()) {
            digitalWrite(msg->pin, msg->state ? HIGH : LOW);
        }
    }

    void SolenoidController::addSolenoidPin(const modelec_interface::msg::Solenoid::SharedPtr msg) {
        solenoid_pin_.insert(msg->pin);
        pinMode(msg->pin, OUTPUT);
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Modelec::SolenoidController>());
    rclcpp::shutdown();
    return 0;
    
}
