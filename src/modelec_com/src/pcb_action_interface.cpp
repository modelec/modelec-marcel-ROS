#include <modelec_com/pcb_action_interface.hpp>
#include <modelec_utils/utils.hpp>
#include <fmt/core.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <modelec_utils/config.hpp>
#include <algorithm>

namespace Modelec
{
    PCBActionInterface::PCBActionInterface() : Node("pcb_action_interface")
    {
        std::string config_path = ament_index_cpp::get_package_share_directory("modelec_strat") + "/data/config.xml";
        if (!Config::load(config_path))
        {
            RCLCPP_ERROR(get_logger(), "Failed to load config file: %s", config_path.c_str());
        }

        std::string action_path = ament_index_cpp::get_package_share_directory("modelec_strat") + "/data/action.xml";
        if (!Config::load(action_path))
        {
            RCLCPP_ERROR(get_logger(), "Failed to load config file: %s", action_path.c_str());
        }

        auto serial_port = Config::get<std::string>("config.usb.action.port", "/dev/ttyUSB0");
        auto baudrate = Config::get<int>("config.usb.action.baudrate", 115200);
        auto timed_servo_timer_ms = Config::get<int>("config.timer.action.timed_servo.ms", TIMER_SERVO_TIMED_MS);

        RCLCPP_INFO(this->get_logger(), "Starting PCB Odometry Interface on port %s with baudrate %ld", serial_port.c_str(), baudrate);

        asc_get_sub_ = this->create_subscription<modelec_interfaces::msg::ActionAscPos>(
            "action/get/asc", 10,
            [this](const modelec_interfaces::msg::ActionAscPos::SharedPtr)
            {
                GetData("ASC", {"POS"});
            });

        servo_get_sub_ = this->create_subscription<modelec_interfaces::msg::ActionServoPosArray>(
            "action/get/servo", 10,
            [this](const modelec_interfaces::msg::ActionServoPosArray::SharedPtr msg)
            {
                std::string data = "GET;SERVO;POS;" + std::to_string(msg->items.size()) + ";";
                for (const auto& item : msg->items) {
                    data += std::to_string(item.id) + ";";
                }
                data += "\n";

                SendToPCB(data);
            });

        relay_get_sub_ = this->create_subscription<modelec_interfaces::msg::ActionRelayStateArray>(
            "action/get/relay", 10,
            [this](const modelec_interfaces::msg::ActionRelayStateArray::SharedPtr msg)
            {
                std::string data = "GET;RELAY;STATE;" + std::to_string(msg->items.size()) + ";";
                for (const auto& item : msg->items) {
                    data += std::to_string(item.id) + ";";
                }
                data += "\n";

                SendToPCB(data);
            });

        asc_get_res_pub_ = this->create_publisher<modelec_interfaces::msg::ActionAscPos>(
            "action/get/asc/res", 10);
        servo_get_res_pub_ = this->create_publisher<modelec_interfaces::msg::ActionServoPosArray>(
            "action/get/servo/res", 10);
        relay_get_res_pub_ = this->create_publisher<modelec_interfaces::msg::ActionRelayStateArray>(
            "action/get/relay/res", 10);

        asc_set_sub_ = this->create_subscription<modelec_interfaces::msg::ActionAscPos>(
            "action/set/asc", 10,
            [this](const modelec_interfaces::msg::ActionAscPos::SharedPtr msg)
            {
                SendOrder("ASC", {std::to_string(msg->pos), std::to_string(msg->value)});
            });

        asc_set_res_pub_ = this->create_publisher<modelec_interfaces::msg::ActionAscPos>(
            "action/set/asc/res", 10);

        asc_move_sub_ = this->create_subscription<modelec_interfaces::msg::ActionAscPos>(
            "action/move/asc", 10,
            [this](const modelec_interfaces::msg::ActionAscPos::SharedPtr msg)
            {
                SendMove("ASC", {std::to_string(msg->pos)});
            });

        servo_move_sub_ = this->create_subscription<modelec_interfaces::msg::ActionServoPosArray>(
            "action/move/servo", 10,
            [this](const modelec_interfaces::msg::ActionServoPosArray::SharedPtr msg)
            {
                std::string data = "MOV;SERVO;" + std::to_string(msg->items.size()) + ";";
                for (const auto& item : msg->items)
                {
                    data += std::to_string(item.id) + ";" + fmt::format("{:.3f}", item.angle) + ";";
                }
                data += "\n";

                SendToPCB(data);
            });

        relay_move_sub_ = this->create_subscription<modelec_interfaces::msg::ActionRelayStateArray>(
            "action/move/relay", 10,
            [this](const modelec_interfaces::msg::ActionRelayStateArray::SharedPtr msg)
            {
                std::string data = "MOV;RELAY;STATE;" + std::to_string(msg->items.size()) + ";";
                for (const auto& item : msg->items)
                {
                    data += std::to_string(item.id) + ";" + std::to_string(item.state) + ";";
                }
                data += "\n";

                SendToPCB(data);
            });

        asc_move_res_pub_ = this->create_publisher<modelec_interfaces::msg::ActionAscPos>(
            "action/move/asc/res", 10);

        servo_move_res_pub_ = this->create_publisher<modelec_interfaces::msg::ActionServoPosArray>(
            "action/move/servo/res", 10);

        relay_move_res_pub_ = this->create_publisher<modelec_interfaces::msg::ActionRelayStateArray>(
            "action/move/relay/res", 10);

        tir_start_pub_ = this->create_publisher<std_msgs::msg::Empty>(
            "action/tir/start", 10);

        tir_start_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "action/tir/start/res", 10,
            [this](const std_msgs::msg::Empty::SharedPtr)
            {
                RespondEvent("TIR", {"START"});
            });

        tir_arm_pub_ = this->create_publisher<std_msgs::msg::Empty>(
            "action/tir/arm", 10);

        tir_arm_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "action/tir/arm/res", 10,
            [this](const std_msgs::msg::Empty::SharedPtr)
            {
                RespondEvent("TIR", {"ARM"});
            });

        tir_disarm_pub_ = this->create_publisher<std_msgs::msg::Empty>(
            "action/tir/disarm", 10);

        tir_disarm_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "action/tir/disarm/res", 10,
            [this](const std_msgs::msg::Empty::SharedPtr)
            {
                RespondEvent("TIR", {"DIS"});
            });

        tir_arm_set_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "action/tir/arm/set", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg)
            {
                SendOrder("TIR", {"ARM", msg->data ? "1" : "0"});
            });


        servo_move_timed_sub_ = this->create_subscription<modelec_interfaces::msg::ActionServoTimedArray>(
            "action/move/servo/timed", 10,
            [this](const modelec_interfaces::msg::ActionServoTimedArray::SharedPtr msg)
            {
                rclcpp::Time now = this->now();

                for (const auto& item : msg->items)
                {
                    ServoTimedSet servo_timed_set;
                    servo_timed_set.servo_timed = item;
                    servo_timed_set.start_time = now + rclcpp::Duration::from_seconds(item.delay_s);
                    servo_timed_set.end_time = now + rclcpp::Duration::from_seconds(item.duration_s + item.delay_s);
                    servo_timed_set.active = true;

                    RCLCPP_DEBUG(this->get_logger(), "Scheduled timed move for Servo ID %d from %.3f to %.3f over %.3f seconds",
                                 item.id, item.start_angle, item.end_angle, item.duration_s);

                    servo_timed_buffer_.push_back(servo_timed_set);
                }
            });

        servo_move_timed_res_pub_ = this->create_publisher<modelec_interfaces::msg::ActionServoTimedArray>(
            "action/move/servo/timed/res", 10);

		servo_timed_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(timed_servo_timer_ms),
            [this]()
            {
                if (servo_timed_buffer_.empty()) {
                    return;
                }

                rclcpp::Time now = this->now();

                modelec_interfaces::msg::ActionServoTimedArray servo_timed_msg;

				std::vector<std::pair<int, double>> to_send;

                for (auto& servo_timed_set : servo_timed_buffer_)
                {
                    if (servo_timed_set.active && now.nanoseconds() < servo_timed_set.start_time.nanoseconds())
                    {
                        continue;
                    }
                    if (servo_timed_set.active && now.nanoseconds() >= servo_timed_set.end_time.nanoseconds())
                    {
                        RCLCPP_DEBUG(this->get_logger(), "Timed move for Servo ID %d completed. Setting to final angle %.3f",
                                     servo_timed_set.servo_timed.id, servo_timed_set.servo_timed.end_angle);

                        to_send.emplace_back(servo_timed_set.servo_timed.id, servo_timed_set.servo_timed.end_angle);

                        servo_timed_set.active = false;

                        servo_timed_msg.items.push_back(servo_timed_set.servo_timed);
                    }
                    else if (servo_timed_set.active)
                    {
                        double elapsed = (now - servo_timed_set.start_time).seconds();
                        double duration = (servo_timed_set.end_time - servo_timed_set.start_time).seconds();
                        double progress = elapsed / duration;

                        RCLCPP_DEBUG(this->get_logger(), "Servo ID %d progress: %.3f | %.3f %.3f",
                            servo_timed_set.servo_timed.id, progress, elapsed, duration);

                        double intermediate_angle = servo_timed_set.servo_timed.start_angle +
                            progress * (servo_timed_set.servo_timed.end_angle - servo_timed_set.servo_timed.start_angle);

						to_send.emplace_back(servo_timed_set.servo_timed.id, intermediate_angle);
                    }
                }

                if (!to_send.empty())
                {
                    std::string data = "MOV;SERVO;" + std::to_string(to_send.size()) + ";";

                    for (const auto& [id, angle] : to_send)
                    {
                        data += std::to_string(id) + ";" + fmt::format("{:.3f}", angle) + ";";
                    }

                    data += "\n";

                    SendToPCB(data);
                }

                if (!servo_timed_msg.items.empty())
                {
                    servo_timed_msg.success = true;
                    servo_move_timed_res_pub_->publish(servo_timed_msg);
                }

                servo_timed_buffer_.erase(
                std::remove_if(
                    servo_timed_buffer_.begin(),
                    servo_timed_buffer_.end(),
                    [&](const ServoTimedSet& s)
                    {
                        return s.active == false;
                    }),
                servo_timed_buffer_.end());


			});

        this->open(baudrate, serial_port, MAX_MESSAGE_LEN);

        servo_value_ = Config::get<std::unordered_map<int, double>>("action.init.servo");

        std::string data = "MOV;SERVO;" + std::to_string(servo_value_.size()) + ";";

        for (auto & [id, v] : servo_value_)
        {
            data += std::to_string(id) + ";" + fmt::format("{:.3f}", v) + ";";
        }

		data += "\n";

        SendToPCB(data);
    }

    PCBActionInterface::~PCBActionInterface()
    {
    }

    void PCBActionInterface::read(const std::string& msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg.c_str());
        RCLCPP_INFO_ONCE(this->get_logger(), "Received message: '%s'", trim(msg).c_str());
        RCLCPP_DEBUG_SKIPFIRST(this->get_logger(), "Received message: '%s'", trim(msg).c_str());
        std::vector<std::string> tokens = split(trim(msg), ';');

        if (tokens.size() < 2)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid message format");
            return;
        }

        if (tokens[0] == "SET")
        {
            if (tokens[1] == "ASC")
            {
                asc_state_ = std::stoi(tokens[3]);

                modelec_interfaces::msg::ActionAscPos asc_msg;
                asc_msg.pos = asc_state_;
                asc_msg.value = asc_value_mapper_[asc_state_];
                asc_msg.success = true;
                asc_get_res_pub_->publish(asc_msg);
            }
            else if (tokens[1] == "SERVO")
            {
                int n = std::stoi(tokens[2]);
                modelec_interfaces::msg::ActionServoPosArray servo_msg;
                servo_msg.items.resize(n);

                for (int i = 0; i < n; ++i)
                {
                    int servo_id = std::stoi(tokens[3 + i * 2]);
                    int angle = std::stoi(tokens[4 + i * 2]);

                    servo_msg.items[i].id = servo_id;
                    servo_msg.items[i].angle = angle;
                    servo_msg.items[i].success = true;
                }
                servo_msg.success = true;

                servo_get_res_pub_->publish(servo_msg);
            }
            else if (tokens[1] == "RELAY")
            {
                int n = std::stoi(tokens[2]);
                modelec_interfaces::msg::ActionRelayStateArray relay_msg;
                relay_msg.items.resize(n);

                for (int i = 0; i < n; ++i)
                {
                    int relay_id = std::stoi(tokens[3 + i * 2]);
                    bool state = (tokens[4 + i * 2] == "1");
                    relay_value_[relay_id] = state;

                    relay_msg.items[i].id = relay_id;
                    relay_msg.items[i].state = state;
                    relay_msg.items[i].success = true;
                }
                relay_msg.success = true;

                relay_get_res_pub_->publish(relay_msg);
            }
        }
        else if (tokens[0] == "OK")
        {
            // TODO rework this part with the pcb protocol
            if (tokens[1] == "ASC")
            {
                if (tokens.size() == 4)
                {
                    int pos = std::stoi(tokens[2]);
                    int v = std::stoi(tokens[3]);
                    asc_value_mapper_[pos] = v;

                    modelec_interfaces::msg::ActionAscPos asc_msg;
                    asc_msg.pos = pos;
                    asc_msg.value = v;
                    asc_msg.success = true;
                    asc_set_res_pub_->publish(asc_msg);
                }
                else
                {
                    asc_state_ = std::stoi(tokens[2]);

                    modelec_interfaces::msg::ActionAscPos asc_msg;
                    asc_msg.pos = asc_state_;
                    asc_msg.success = true;
                    asc_move_res_pub_->publish(asc_msg);
                }
            }
            else if (tokens[1] == "SERVO")
            {
                int n = std::stoi(tokens[2]);
                modelec_interfaces::msg::ActionServoPosArray servo_msg;
                servo_msg.items.resize(n);

                for (int i = 0; i < n; ++i)
                {
                    int servo_id = std::stoi(tokens[3 + i * 2]);
                    float angle = std::stof(tokens[4 + i * 2]);
                    servo_value_[servo_id] = angle;

                    servo_msg.items[i].id = servo_id;
                    servo_msg.items[i].angle = angle;
                    servo_msg.items[i].success = true;
                }
                servo_msg.success = true;

                servo_move_res_pub_->publish(servo_msg);
            }
            else if (tokens[1] == "RELAY")
            {
                int n = std::stoi(tokens[2]);
                modelec_interfaces::msg::ActionRelayStateArray relay_msg;
                relay_msg.items.resize(n);

                for (int i = 0; i < n; ++i)
                {
                    int relay_id = std::stoi(tokens[3 + i * 2]);
                    bool state = (tokens[4 + i * 2] == "1");
                    relay_value_[relay_id] = state;

                    relay_msg.items[i].id = relay_id;
                    relay_msg.items[i].state = state;
                    relay_msg.items[i].success = true;
                }
                relay_msg.success = true;

                relay_move_res_pub_->publish(relay_msg);
            }
            else if (tokens[1] == "TIR")
            {
                // Do nothing for now
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Unknown message format for OK response: '%s'", trim(msg).c_str());
            }
        }
        else if (tokens[0] == "KO")
        {
            if (tokens[1] == "ASC")
            {
                if (tokens.size() == 4)
                {
                    modelec_interfaces::msg::ActionAscPos asc_msg;
                    asc_msg.success = false;
                    asc_set_res_pub_->publish(asc_msg);
                }
                else
                {
                    modelec_interfaces::msg::ActionAscPos asc_msg;
                    asc_msg.success = false;
                    asc_move_res_pub_->publish(asc_msg);
                }
            }
            else if (tokens[1] == "SERVO")
            {
                modelec_interfaces::msg::ActionServoPosArray servo_msg;
                servo_msg.success = false;
                servo_move_res_pub_->publish(servo_msg);
            }
            else if (tokens[1] == "RELAY")
            {
                modelec_interfaces::msg::ActionRelayStateArray relay_msg;
                relay_msg.success = false;
                relay_move_res_pub_->publish(relay_msg);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Unknown message format: '%s'", trim(msg).c_str());
            }
        }
        else if (tokens[0] == "EVENT")
        {
            if (tokens[1] == "TIR")
            {
                if (tokens[2] == "START")
                {
                    tir_start_pub_->publish(std_msgs::msg::Empty());
                    RespondEvent(tokens[1], {tokens[2]});
                }
                else if (tokens[2] == "ARM")
                {
                    tir_arm_pub_->publish(std_msgs::msg::Empty());
                    RespondEvent(tokens[1], {tokens[2]});
                }
                else if (tokens[2] == "DIS")
                {
                    tir_disarm_pub_->publish(std_msgs::msg::Empty());
                    RespondEvent(tokens[1], {tokens[2]});
                }
            }
        }
    }

    void PCBActionInterface::SendToPCB(const std::string& data)
    {
        if (IsOk())
        {
            // RCLCPP_INFO(this->get_logger(), "Sending to PCB: %s", trim(data).c_str());
            RCLCPP_INFO_ONCE(this->get_logger(), "Sending to PCB: %s", trim(data).c_str());
            RCLCPP_DEBUG_SKIPFIRST(this->get_logger(), "Sending to PCB: %s", data.c_str());
            this->write(data);
        }
    }

    void PCBActionInterface::SendToPCB(const std::string& order, const std::string& elem,
                                       const std::vector<std::string>& data)
    {
        std::string command = order + ";" + elem;
        for (const auto& d : data)
        {
            command += ";" + d;
        }
        command += "\n";

        SendToPCB(command);
    }

    void PCBActionInterface::GetData(const std::string& elem, const std::vector<std::string>& data)
    {
        SendToPCB("GET", elem, data);
    }

    void PCBActionInterface::SendOrder(const std::string& elem, const std::vector<std::string>& data)
    {
        SendToPCB("SET", elem, data);
    }

    void PCBActionInterface::SendMove(const std::string& elem, const std::vector<std::string>& data)
    {
        SendToPCB("MOV", elem, data);
    }

    void PCBActionInterface::RespondEvent(const std::string& elem, const std::vector<std::string>& data)
    {
        SendToPCB("ACK", elem, data);
    }
}

#ifndef MODELEC_COM_TESTING
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Modelec::PCBActionInterface>();

    // Increase number of threads explicitly!
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 2 /* or more threads! */);

    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
#endif
