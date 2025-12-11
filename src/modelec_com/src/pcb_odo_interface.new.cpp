#include <modelec_com/pcb_odo_interface.new.hpp>
#include <modelec_com/serial_listener.hpp>
#include <modelec_utils/utils.hpp>
#include <modelec_interfaces/srv/add_serial_listener.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <modelec_utils/config.hpp>

namespace Modelec
{
    PCBOdoInterface::PCBOdoInterface() : Node("pcb_odo_interface"), SerialListener()
    {
        declare_parameter<std::string>("serial_port", "/tmp/USB_ODO");
        declare_parameter<int>("baudrate", 115200);
        declare_parameter<std::string>("name", "pcb_odo");

        // Service to create a new serial listener
        auto request = std::make_shared<modelec_interfaces::srv::AddSerialListener::Request>();
        request->name = get_parameter("name").as_string();
        request->bauds = get_parameter("baudrate").as_int();
        request->serial_port = get_parameter("serial_port").as_string();

        this->open(request->name, request->bauds, request->serial_port, MAX_MESSAGE_LEN);

        /*auto client = this->create_client<modelec_interfaces::srv::AddSerialListener>("add_serial_listener");
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
                        },
                        options);

                    pcb_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
                    pcb_executor_->add_callback_group(pcb_callback_group_, this->get_node_base_interface());

                    pcb_executor_thread_ = std::thread([this]()
                    {
                        pcb_executor_->spin();
                    });

                    pcb_publisher_ = this->create_publisher<std_msgs::msg::String>(res->subscriber, 10);

                    isOk = true;

                    SetStart(true);

                    SetPID("THETA", 14, 0, 0);
                    SetPID("POS", 10, 0, 0);
                    SetPID("LEFT", 5, 0, 0);
                    SetPID("RIGHT", 5, 0, 0);
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
        }*/

        odo_pos_publisher_ = this->create_publisher<modelec_interfaces::msg::OdometryPos>(
            "odometry/position", 10);

        odo_get_pos_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "odometry/get/pos", 30, [this](const std_msgs::msg::Empty::SharedPtr)
            {
                if (isOk)
                {
                    GetPos();
                }
            });

        odo_speed_publisher_ = this->create_publisher<modelec_interfaces::msg::OdometrySpeed>(
            "odometry/speed", 10);

        odo_tof_publisher_ = this->create_publisher<modelec_interfaces::msg::OdometryToF>(
            "odometry/tof", 10);

        odo_waypoint_reach_publisher_ = this->create_publisher<modelec_interfaces::msg::OdometryWaypoint>(
            "odometry/waypoint_reach", 10);

        odo_pid_publisher_ = this->create_publisher<modelec_interfaces::msg::OdometryPid>(
            "odometry/get_pid", 10);

        odo_add_waypoint_subscriber_ = this->create_subscription<modelec_interfaces::msg::OdometryWaypoint>(
            "odometry/add_waypoint", 30,
            [this](const modelec_interfaces::msg::OdometryWaypoint::SharedPtr msg)
            {
                AddWaypointCallback(msg);
            });

        odo_add_waypoints_subscriber_ = this->create_subscription<modelec_interfaces::msg::OdometryWaypoints>(
            "odometry/add_waypoints", 30,
            [this](const modelec_interfaces::msg::OdometryWaypoints::SharedPtr msg)
            {
                AddWaypointsCallback(msg);
            });

        odo_set_pos_subscriber_ = this->create_subscription<modelec_interfaces::msg::OdometryPos>(
            "odometry/set_position", 10,
            [this](const modelec_interfaces::msg::OdometryPos::SharedPtr msg)
            {
                SetPosCallback(msg);
            });

        odo_set_pid_subscriber_ = this->create_subscription<modelec_interfaces::msg::OdometryPid>(
            "odometry/set_pid", 10,
            [this](const modelec_interfaces::msg::OdometryPid::SharedPtr msg)
            {
                SetPIDCallback(msg);
            });

        start_odo_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "odometry/start", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg)
            {
                if (msg->data != start_odo_)
                {
                    start_odo_ = msg->data;
                    SendOrder("START", {std::to_string(msg->data)});
                }
            });
    }

    PCBOdoInterface::~PCBOdoInterface()
    {
        SetStart(false);

        if (pcb_executor_)
        {
            pcb_executor_->cancel();
        }
        if (pcb_executor_thread_.joinable())
        {
            pcb_executor_thread_.join();
        }
    }

    void PCBOdoInterface::read(const std::string& msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received from PCB: %s", msg.c_str());
        std::vector<std::string> tokens = split(trim(msg), ';');
        if (tokens.size() < 2)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid message format: %s", msg.c_str());
            return;
        }

        if (tokens[0] == "SET")
        {
            if (tokens[1] == "POS")
            {
                long x = std::stol(tokens[2]);
                long y = std::stol(tokens[3]);
                double theta = std::stod(tokens[4]);

                auto message = modelec_interfaces::msg::OdometryPos();
                message.x = x;
                message.y = y;
                message.theta = theta;

                if (odo_pos_publisher_) {
                    odo_pos_publisher_->publish(message);
                }
            }
            else if (tokens[1] == "SPEED")
            {
                long x = std::stol(tokens[2]);
                long y = std::stol(tokens[3]);
                double theta = std::stod(tokens[4]);

                auto message = modelec_interfaces::msg::OdometrySpeed();
                message.x = x;
                message.y = y;
                message.theta = theta;

                odo_speed_publisher_->publish(message);
            }
            else if (tokens[1] == "DIST")
            {
                int n = std::stoi(tokens[2]);
                long dist = std::stol(tokens[3]);

                auto message = modelec_interfaces::msg::OdometryToF();
                message.n = n;
                message.distance = dist;

                odo_tof_publisher_->publish(message);
            }
            else if (tokens[1] == "WAYPOINT")
            {
                int id = std::stoi(tokens[2]);

                auto message = modelec_interfaces::msg::OdometryWaypoint();
                message.id = id;

                odo_waypoint_reach_publisher_->publish(message);
            }
            else if (tokens[1] == "PID")
            {
                std::string name = tokens[2];
                float p = std::stof(tokens[3]);
                float i = std::stof(tokens[4]);
                float d = std::stof(tokens[5]);

                auto message = modelec_interfaces::msg::OdometryPid();
                message.name = name;
                message.p = p;
                message.i = i;
                message.d = d;

                odo_pid_publisher_->publish(message);
            }
        }
        else if (tokens[0] == "OK")
        {
            if (tokens[1] == "START")
            {
                start_odo_ = tokens[2] == "1";
            }
            else if (tokens[1] == "WAYPOINT")
            {
            }
            else if (tokens[1] == "PID")
            {
            }
            else if (tokens[1] == "POS")
            {
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "PCB response: %s", msg.c_str());
            }
        }
        else if (tokens[0] == "KO")
        {
            if (tokens[1] == "START")
            {
            }
            else if (tokens[1] == "WAYPOINT")
            {
            }

            else if (tokens[1] == "PID")
            {
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "PCB error: %s", msg.c_str());
            }
        }
    }

    void PCBOdoInterface::AddWaypointsCallback(const modelec_interfaces::msg::OdometryWaypoints::SharedPtr msg) {
        AddWaypoints(msg);
    }

    void PCBOdoInterface::AddWaypointCallback(const modelec_interfaces::msg::OdometryWaypoint::SharedPtr msg)
    {
        AddWaypoint(msg);
    }

    void PCBOdoInterface::SetPosCallback(const modelec_interfaces::msg::OdometryPos::SharedPtr msg)
    {
        SetRobotPos(msg);
    }

    void PCBOdoInterface::SetPIDCallback(const modelec_interfaces::msg::OdometryPid::SharedPtr msg)
    {
        SetPID(msg);
    }

    void PCBOdoInterface::SendToPCB(const std::string& data)
    {
        if (pcb_publisher_)
        {
            RCLCPP_DEBUG(this->get_logger(), "Sending to PCB: %s", data.c_str());
            auto message = std_msgs::msg::String();
            message.data = data;
            pcb_publisher_->publish(message);
        }
    }

    void PCBOdoInterface::SendToPCB(const std::string& order, const std::string& elem,
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

    void PCBOdoInterface::GetData(const std::string& elem, const std::vector<std::string>& data)
    {
        SendToPCB("GET", elem, data);
    }

    void PCBOdoInterface::SendOrder(const std::string& elem, const std::vector<std::string>& data)
    {
        SendToPCB("SET", elem, data);
    }

    void PCBOdoInterface::GetPos()
    {
        GetData("POS");
    }

    void PCBOdoInterface::GetSpeed()
    {
        GetData("SPEED");
    }

    void PCBOdoInterface::GetToF(const int& tof)
    {
        GetData("DIST", {std::to_string(tof)});
    }

    void PCBOdoInterface::SetRobotPos(const modelec_interfaces::msg::OdometryPos::SharedPtr msg)
    {
        SetRobotPos(msg->x, msg->y, msg->theta);
    }

    void PCBOdoInterface::SetRobotPos(const long x, const long y, const double theta)
    {
        std::vector<std::string> data = {
            std::to_string(x),
            std::to_string(y),
            std::to_string(theta)
        };

        SendOrder("POS", data);
    }

    void PCBOdoInterface::AddWaypoints(modelec_interfaces::msg::OdometryWaypoints::SharedPtr msg)
    {
        if (!start_odo_)
        {
            SendOrder("START", {std::to_string(true)});
        }

        std::vector<std::string> data;

        for (const auto& wp : msg->waypoints)
        {
            data.push_back(std::to_string(wp.id));
            data.push_back(std::to_string(wp.is_end));
            data.push_back(std::to_string(wp.x));
            data.push_back(std::to_string(wp.y));
            data.push_back(std::to_string(wp.theta));
       }

        SendOrder("WAYPOINT", data);
    }

    void PCBOdoInterface::AddWaypoint(
        const modelec_interfaces::msg::OdometryWaypoint::SharedPtr msg)
    {
        AddWaypoint(msg->id, msg->is_end, msg->x, msg->y, msg->theta);
    }

    void PCBOdoInterface::AddWaypoint(const int index, const bool IsStopPoint, const long x, const long y,
                                      const double theta)
    {
        if (!start_odo_)
        {
            SendOrder("START", {std::to_string(true)});
        }

        std::vector<std::string> data = {
            std::to_string(index),
            std::to_string(IsStopPoint),
            std::to_string(x),
            std::to_string(y),
            std::to_string(theta)
        };

        SendOrder("WAYPOINT", data);
    }

    void PCBOdoInterface::SetStart(const modelec_interfaces::msg::OdometryStart::SharedPtr msg)
    {
        SetStart(msg->start);
    }

    void PCBOdoInterface::SetStart(bool start)
    {
        SendOrder("START", {std::to_string(start)});
    }

    void PCBOdoInterface::GetPID()
    {
        GetData("PID");
    }

    void PCBOdoInterface::SetPID(const modelec_interfaces::msg::OdometryPid::SharedPtr msg)
    {
        SetPID(msg->name, msg->p, msg->i, msg->d);
    }

    void PCBOdoInterface::SetPID(std::string name, float p, float i, float d, std::optional<float> min, std::optional<float> max)
    {
        std::vector<std::string> data = {
            name,
            std::to_string(p),
            std::to_string(i),
            std::to_string(d)
        };

        if (min.has_value())
        {
            data.push_back(std::to_string(min.value()));
        }

        if (max.has_value())
        {
            data.push_back(std::to_string(max.value()));
        }

        SendOrder("PID", data);
    }
} // Modelec

#ifndef MODELEC_COM_TESTING
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Modelec::PCBOdoInterface>();

    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 2);

    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
#endif