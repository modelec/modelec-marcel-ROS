#include <modelec/button_gpio_controller.hpp>

namespace Modelec {
  ButtonGpioController::ButtonGpioController() : Node("button_gpio_controller") {
    if (wiringPiSetupGpio() == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize GPIO.");
        rclcpp::shutdown();
    }

    new_button_service_ = create_service<modelec_interface::srv::AddButton>("add_button", std::bind(&ButtonGpioController::new_button, this, std::placeholders::_1, std::placeholders::_2));
    button_server_ = create_service<modelec_interface::srv::Button>("button", std::bind(&ButtonGpioController::check_button, this, std::placeholders::_1, std::placeholders::_2));
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      for (auto& button : buttons_) {
        modelec_interface::msg::Button msg;
        msg.pin = button.second.pin;
        msg.state = digitalRead(button.second.pin) == LOW;
        button.second.publisher->publish(msg);
      }
    });
  }

  void ButtonGpioController::new_button(const std::shared_ptr<modelec_interface::srv::AddButton::Request> request, std::shared_ptr<modelec_interface::srv::AddButton::Response> response) {
    if (buttons_.find(request->pin) != buttons_.end()) {
      response->success = false;
      return;
    }

    Button button;
    button.pin = request->pin;
    button.name = request->name;
    button.publisher = create_publisher<modelec_interface::msg::Button>("button/" + request->name, 10);

    buttons_.insert({request->pin, button});

    pinMode(request->pin, INPUT);
    pullUpDnControl(request->pin, PUD_UP);

    response->success = true;
  }

  void ButtonGpioController::check_button(const std::shared_ptr<modelec_interface::srv::Button::Request> request, std::shared_ptr<modelec_interface::srv::Button::Response> response) {
    if (buttons_.find(request->pin) == buttons_.end()) {
      response->status = false;
      return;
    }

    response->status = digitalRead(request->pin) == LOW;
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Modelec::ButtonGpioController>());
  rclcpp::shutdown();
  return 0;
}
