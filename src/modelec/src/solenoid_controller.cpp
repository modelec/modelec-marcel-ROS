#include <modelec/solenoid_controller.hpp>
#include <wiringPi.h>

namespace Modelec {
    SolenoidController::SolenoidController() : Node("solenoid_controller") {
        wiringPiSetup();

        solenoid_subscriber_ = this->create_subscription<modelec_interface::msg::Solenoid>(
            "solenoid", 10, std::bind(&SolenoidController::activateSolenoid, this, std::placeholders::_1));

        add_solenoid_service_ = this->create_service<modelec_interface::srv::NewSolenoid>(
            "add_solenoid", std::bind(&SolenoidController::addSolenoid, this, std::placeholders::_1, std::placeholders::_2));
    }

    void SolenoidController::activateSolenoid(const modelec_interface::msg::Solenoid::SharedPtr msg) {
        // Activate solenoid
        if (solenoid_pin_.find(msg->pin) != solenoid_pin_.end()) {
            digitalWrite(msg->pin, msg->state ? HIGH : LOW);
        }
    }

    void SolenoidController::addSolenoid(const std::shared_ptr<modelec_interface::srv::NewSolenoid::Request> request,
                                         std::shared_ptr<modelec_interface::srv::NewSolenoid::Response> response) {
        if (solenoid_pin_.find(request->pin) != solenoid_pin_.end()) {
            response->success = false;
            return;
        }

        pinMode(request->pin, OUTPUT);
        digitalWrite(request->pin, LOW);
        solenoid_pin_.insert(request->pin);

        response->success = true;
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Modelec::SolenoidController>());
    rclcpp::shutdown();
    return 0;
    
}
