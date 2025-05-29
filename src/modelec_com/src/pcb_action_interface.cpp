#include <ament_index_cpp/get_package_share_directory.hpp>
#include <modelec_com/pcb_action_interface.hpp>
#include <modelec_utils/config.hpp>
#include <modelec_utils/utils.hpp>

namespace Modelec
{
    PCBActionInterface::PCBActionInterface() : Node("pcb_action_interface")
    {
        // Service to create a new serial listener
        std::string config_path = ament_index_cpp::get_package_share_directory("modelec_strat") + "/data/config.xml";
        if (!Config::load(config_path))
        {
            RCLCPP_ERROR(get_logger(), "Failed to load config file: %s", config_path.c_str());
        }

        auto request = std::make_shared<modelec_interfaces::srv::AddSerialListener::Request>();
        request->name = Config::get<std::string>("config.usb.pcb.pcb_action.name", "pcb_action");
        request->bauds = Config::get<int>("config.usb.pcb.pcb_action.baudrate", 115200);
        request->serial_port = Config::get<std::string>("config.usb.pcb.pcb_action.port", "/dev/ttyUSB0");

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
                    pcb_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

                    rclcpp::SubscriptionOptions options;
                    options.callback_group = pcb_callback_group_;

                    pcb_subscriber_ = this->create_subscription<std_msgs::msg::String>(
                        res->publisher, 10,
                        [this](const std_msgs::msg::String::SharedPtr msg)
                        {
                            PCBCallback(msg);
                        }, options);

                    pcb_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
                    pcb_executor_->add_callback_group(pcb_callback_group_, this->get_node_base_interface());

                    pcb_executor_thread_ = std::thread([this]()
                    {
                        pcb_executor_->spin();
                    });

                    pcb_publisher_ = this->create_publisher<std_msgs::msg::String>(res->subscriber, 10);

                    isOk = true;
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to add serial listener");
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to ask for a serial listener");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }

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
                RCLCPP_INFO(this->get_logger(), "TIR arm set to: %s", msg->data ? "true" : "false");
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
            SendMove("SERVO" + std::to_string(id), {"POS" + std::to_string(v)});
        }

        relay_value_ = {
            {0, false},
            {1, false},
            {2, false},
        };

        for (auto & [id, v] : relay_value_)
        {
            SendMove("RELAY" + std::to_string(id), {std::to_string(v)});
        }

        SendOrder("TIR", {"ARM", "1"});
    }

    PCBActionInterface::~PCBActionInterface()
    {
        if (pcb_executor_)
        {
            pcb_executor_->cancel();
        }
        if (pcb_executor_thread_.joinable())
        {
            pcb_executor_thread_.join();
        }
    }

    void PCBActionInterface::PCBCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());
        std::vector<std::string> tokens = split(trim(msg->data), ';');

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
                RCLCPP_WARN(this->get_logger(), "Unknown message format for OK response: '%s'", msg->data.c_str());
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
                RCLCPP_WARN(this->get_logger(), "Unknown message format: '%s'", msg->data.c_str());
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

    void PCBActionInterface::SendToPCB(const std::string& data) const
    {
        if (pcb_publisher_)
        {
            RCLCPP_INFO(this->get_logger(), "Sending to PCB: '%s'", data.c_str());

            auto message = std_msgs::msg::String();
            message.data = data;
            pcb_publisher_->publish(message);
        }
    }

    void PCBActionInterface::SendToPCB(const std::string& order, const std::string& elem,
                                       const std::vector<std::string>& data) const
    {
        std::string command = order + ";" + elem;
        for (const auto& d : data)
        {
            command += ";" + d;
        }
        command += "\n";

        SendToPCB(command);
    }

    void PCBActionInterface::GetData(const std::string& elem, const std::vector<std::string>& data) const
    {
        SendToPCB("GET", elem, data);
    }

    void PCBActionInterface::SendOrder(const std::string& elem, const std::vector<std::string>& data) const
    {
        SendToPCB("SET", elem, data);
    }

    void PCBActionInterface::SendMove(const std::string& elem, const std::vector<std::string>& data) const
    {
        SendToPCB("MOV", elem, data);
    }

    void PCBActionInterface::RespondEvent(const std::string& elem, const std::vector<std::string>& data) const
    {
        SendToPCB("ACK", elem, data);
    }
}

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
