#include <modelec_com/pcb_action_interface.new.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <modelec_utils/config.hpp>
#include <modelec_utils/utils.hpp>

namespace Modelec
{
    PCBActionInterface::PCBActionInterface() : Node("pcb_action_interface"), SerialListener()
    {
        // Service to create a new serial listener
        declare_parameter<std::string>("serial_port", "/dev/USB_ACTION");
        declare_parameter<int>("baudrate", 115200);
        declare_parameter<std::string>("name", "pcb_action");

        auto request = std::make_shared<modelec_interfaces::srv::AddSerialListener::Request>();
        request->name = get_parameter("name").as_string();
        request->bauds = get_parameter("baudrate").as_int();
        request->serial_port = get_parameter("serial_port").as_string();

        this->open(request->name, request->bauds, request->serial_port, MAX_MESSAGE_LEN);

        asc_get_sub_ = this->create_subscription<modelec_interfaces::msg::ActionAscPos>(
            "action/get/asc", 10,
            [this](const modelec_interfaces::msg::ActionAscPos::SharedPtr)
            {
                GetData("ASC", {"POS"});
            });

        servo_get_sub_ = this->create_subscription<modelec_interfaces::msg::ActionServoPos>(
            "action/get/servo", 10,
            [this](const modelec_interfaces::msg::ActionServoPos::SharedPtr msg)
            {
                GetData("SERVO" + std::to_string(msg->id), {"POS"});
            });

        relay_get_sub_ = this->create_subscription<modelec_interfaces::msg::ActionRelayState>(
            "action/get/relay", 10,
            [this](const modelec_interfaces::msg::ActionRelayState::SharedPtr msg)
            {
                GetData("RELAY" + std::to_string(msg->id), {"STATE"});
            });

        asc_get_res_pub_ = this->create_publisher<modelec_interfaces::msg::ActionAscPos>(
            "action/get/asc/res", 10);
        servo_get_res_pub_ = this->create_publisher<modelec_interfaces::msg::ActionServoPos>(
            "action/get/servo/res", 10);
        relay_get_res_pub_ = this->create_publisher<modelec_interfaces::msg::ActionRelayState>(
            "action/get/relay/res", 10);

        asc_set_sub_ = this->create_subscription<modelec_interfaces::msg::ActionAscPos>(
            "action/set/asc", 10,
            [this](const modelec_interfaces::msg::ActionAscPos::SharedPtr msg)
            {
                SendOrder("ASC", {std::to_string(msg->pos), std::to_string(msg->value)});
            });

        servo_set_sub_ = this->create_subscription<modelec_interfaces::msg::ActionServoPos>(
            "action/set/servo", 10,
            [this](const modelec_interfaces::msg::ActionServoPos::SharedPtr msg)
            {
                SendOrder("SERVO" + std::to_string(msg->id), {
                              "POS" + std::to_string(msg->pos), std::to_string(static_cast<int>(msg->angle * 100))
                          });
            });

        asc_set_res_pub_ = this->create_publisher<modelec_interfaces::msg::ActionAscPos>(
            "action/set/asc/res", 10);

        servo_set_res_pub_ = this->create_publisher<modelec_interfaces::msg::ActionServoPos>(
            "action/set/servo/res", 10);

        asc_move_sub_ = this->create_subscription<modelec_interfaces::msg::ActionAscPos>(
            "action/move/asc", 10,
            [this](const modelec_interfaces::msg::ActionAscPos::SharedPtr msg)
            {
                SendMove("ASC", {std::to_string(msg->pos)});
            });

        servo_move_sub_ = this->create_subscription<modelec_interfaces::msg::ActionServoPos>(
            "action/move/servo", 10,
            [this](const modelec_interfaces::msg::ActionServoPos::SharedPtr msg)
            {
                SendMove("SERVO" + std::to_string(msg->id), {"POS" + std::to_string(msg->pos)});
            });

        relay_move_sub_ = this->create_subscription<modelec_interfaces::msg::ActionRelayState>(
            "action/move/relay", 10,
            [this](const modelec_interfaces::msg::ActionRelayState::SharedPtr msg)
            {
                SendMove("RELAY" + std::to_string(msg->id), {std::to_string(msg->state)});
            });

        asc_move_res_pub_ = this->create_publisher<modelec_interfaces::msg::ActionAscPos>(
            "action/move/asc/res", 10);

        servo_move_res_pub_ = this->create_publisher<modelec_interfaces::msg::ActionServoPos>(
            "action/move/servo/res", 10);

        relay_move_res_pub_ = this->create_publisher<modelec_interfaces::msg::ActionRelayState>(
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


        // TODO : check for real value there
        asc_value_mapper_ = {
            {0, 0},
            {1, 100},
            {2, 200},
            {3, 300}
        };
        /*for (auto & [id, v] : asc_value_mapper_)
        {
            SendOrder("ASC", {std::to_string(id), std::to_string(v)});
        }*/

        asc_state_ = 3;

        // SendMove("ASC", {std::to_string(asc_state_)});

        servo_pos_mapper_ = {
            {0, {{0, 0.55}, {1, 0}}},
            {1, {{0, 0}, {1, 0.4}}},
            {2, {{0, M_PI_2}}},
            {3, {{0, M_PI_2}}},
            {4, {{0, 1.25}, {1, 0.45}}},
            {5, {{0, 0}, {1, M_PI}}},
        };

        for (auto & [id, v] : servo_pos_mapper_)
        {
            if (id == 5) continue;

            for (auto & [key, angle] : v)
            {
                rclcpp::sleep_for(std::chrono::milliseconds(100));

                SendOrder("SERVO" + std::to_string(id), {"POS" + std::to_string(key), std::to_string(static_cast<int>(angle * 100))});
            }
        }

        servo_value_ = {
            {0, 1},
            {1, 1},
            {2, 0},
            {3, 0},
            {4, 1},
            {5, 0}
        };

        for (auto & [id, v] : servo_value_)
        {
            rclcpp::sleep_for(std::chrono::milliseconds(100));

            SendMove("SERVO" + std::to_string(id), {"POS" + std::to_string(v)});
        }

        relay_value_ = {
            {1, false},
            {2, false},
            {3, false},
        };

        for (auto & [id, v] : relay_value_)
        {
            rclcpp::sleep_for(std::chrono::milliseconds(100));

            SendMove("RELAY" + std::to_string(id), {std::to_string(v)});
        }

        rclcpp::sleep_for(std::chrono::milliseconds(100));

        SendOrder("TIR", {"ARM", "1"});
    }

    PCBActionInterface::~PCBActionInterface()
    {
    }

    void PCBActionInterface::read(const std::string& msg)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "Received message: '%s'", msg.c_str());
        RCLCPP_DEBUG_SKIPFIRST(this->get_logger(), "Received message: '%s'", msg.c_str());
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
            else if (startsWith(tokens[1], "SERVO"))
            {
                int servo_id = std::stoi(tokens[1].substr(5));
                int v = std::stoi(tokens[3]);
                servo_value_[servo_id] = v;

                modelec_interfaces::msg::ActionServoPos servo_msg;
                servo_msg.id = servo_id;
                servo_msg.pos = v;
                servo_msg.angle = servo_pos_mapper_[servo_id][v];
                servo_msg.success = true;
                servo_get_res_pub_->publish(servo_msg);
            }
            else if (startsWith(tokens[1], "RELAY"))
            {
                int relay_id = std::stoi(tokens[1].substr(5));
                bool state = (tokens[3] == "1");
                relay_value_[relay_id] = state;

                modelec_interfaces::msg::ActionRelayState relay_msg;
                relay_msg.id = relay_id;
                relay_msg.state = state;
                relay_msg.success = true;
                relay_get_res_pub_->publish(relay_msg);
            }
        }
        else if (tokens[0] == "OK")
        {
            if (tokens.size() == 4)
            {
                if (tokens[1] == "ASC")
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
                else if (startsWith(tokens[1], "SERVO"))
                {
                    int servo_id = std::stoi(tokens[1].substr(5));
                    int key = std::stoi(tokens[2].substr(3));
                    int v = std::stoi(tokens[3]);
                    servo_pos_mapper_[servo_id][key] = v;

                    modelec_interfaces::msg::ActionServoPos servo_msg;
                    servo_msg.id = servo_id;
                    servo_msg.pos = key;
                    servo_msg.angle = v;
                    servo_msg.success = true;
                    servo_set_res_pub_->publish(servo_msg);
                }
            }
            else if (tokens.size() == 3)
            {
                if (tokens[1] == "ASC")
                {
                    asc_state_ = std::stoi(tokens[2]);

                    modelec_interfaces::msg::ActionAscPos asc_msg;
                    asc_msg.pos = asc_state_;
                    asc_msg.success = true;
                    asc_move_res_pub_->publish(asc_msg);
                }
                else if (startsWith(tokens[1], "SERVO"))
                {
                    int servo_id = std::stoi(tokens[1].substr(5));
                    int key = std::stoi(tokens[2].substr(3));
                    servo_value_[servo_id] = key;

                    modelec_interfaces::msg::ActionServoPos servo_msg;
                    servo_msg.id = servo_id;
                    servo_msg.pos = key;
                    servo_msg.success = true;
                    servo_move_res_pub_->publish(servo_msg);
                }
                else if (startsWith(tokens[1], "RELAY"))
                {
                    int relay_id = std::stoi(tokens[1].substr(5));
                    bool state = (tokens[2] == "1");
                    relay_value_[relay_id] = state;

                    modelec_interfaces::msg::ActionRelayState relay_msg;
                    relay_msg.id = relay_id;
                    relay_msg.state = state;
                    relay_msg.success = true;
                    relay_move_res_pub_->publish(relay_msg);
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Unknown message format for OK response: '%s'", msg.c_str());
            }
        }
        else if (tokens[0] == "KO")
        {
            if (tokens.size() == 4)
            {
                if (tokens[1] == "ASC")
                {
                    modelec_interfaces::msg::ActionAscPos asc_msg;
                    asc_msg.success = false;
                    asc_set_res_pub_->publish(asc_msg);
                }
                else if (startsWith(tokens[1], "SERVO"))
                {
                    modelec_interfaces::msg::ActionServoPos servo_msg;
                    servo_msg.success = false;
                    servo_set_res_pub_->publish(servo_msg);
                }
            }
            else if (tokens.size() == 3)
            {
                if (tokens[1] == "ASC")
                {
                    modelec_interfaces::msg::ActionAscPos asc_msg;
                    asc_msg.success = false;
                    asc_move_res_pub_->publish(asc_msg);
                }
                else if (startsWith(tokens[1], "SERVO"))
                {
                    modelec_interfaces::msg::ActionServoPos servo_msg;
                    servo_msg.success = false;
                    servo_move_res_pub_->publish(servo_msg);
                }
                else if (startsWith(tokens[1], "RELAY"))
                {
                    modelec_interfaces::msg::ActionRelayState relay_msg;
                    relay_msg.success = false;
                    relay_move_res_pub_->publish(relay_msg);
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Unknown message format: '%s'", msg.c_str());
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
            RCLCPP_DEBUG(this->get_logger(), "Sending to PCB: %s", data.c_str());
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
